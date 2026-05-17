#!/bin/bash
# ===========================================================
# run_full_test.sh — 一键运行 BT 仿真测试
# 启动 roscore、加载参数、启动 TF/BT/Simulator/Logger
# 约 115 秒后自动结束并输出统计
# ===========================================================
set -euo pipefail

SIM_DIR="$(cd "$(dirname "$0")" && pwd)"
# Allow override from environment and fall back to workspace inferred from script path.
DEFAULT_WS_ROOT="$(cd "${SIM_DIR}/../../../../.." && pwd)"
WS_ROOT="${WS_ROOT:-${DEFAULT_WS_ROOT}}"
BT_BIN="${WS_ROOT}/devel/.private/rm_behavior_tree/lib/rm_behavior_tree/rm_behavior_tree"
YAML="${SIM_DIR}/../test/sentry.yaml"
XML="${SIM_DIR}/../config/main_tree.xml"
BT_LOG="/tmp/bt_sim_output_$$.log"
SIM_LOG="/tmp/bt_sim_driver_output_$$.log"
CAP_LOG="/tmp/bt_cap_monitor_output_$$.log"
SCENARIO="${1:-full_match}"
SPEED="${2:-3.0}"
if [[ -n "${TEST_DURATION:-}" ]]; then
    DURATION="${TEST_DURATION}"
else
    case "${SCENARIO}" in
        full_match) DURATION=120 ;;  # 300/3 + 余量
        rulebook_full_chain) DURATION=130 ;;  # 覆盖到赛后 AbnormalStill
        rulebook_full_chain_dirty) DURATION=140 ;;
        chase_gate_020b) DURATION=20 ;;
        control_area_dirty_gate) DURATION=22 ;;
        posture_transitions) DURATION=55 ;;
        capacitor_middle_area_demo|capacitor_middle_area_disabled) DURATION=55 ;;
        enemy_invincible_transition) DURATION=28 ;;
        enemy_revive_invincible_dirty_on|enemy_revive_invincible_dirty_off) DURATION=24 ;;
        weak_rfid_jitter|death_revive) DURATION=32 ;;
        hp_urgent) DURATION=32 ;;
        *) DURATION=45 ;;
    esac
fi

TEMP_ROS_HOME=""
if [[ -z "${ROS_HOME:-}" ]]; then
    TEMP_ROS_HOME="$(mktemp -d /tmp/rm_bt_ros_XXXXXX)"
    export ROS_HOME="${TEMP_ROS_HOME}"
else
    export ROS_HOME
fi
mkdir -p "${ROS_HOME}"

pick_free_port() {
    local candidate
    local attempt
    for attempt in $(seq 1 20); do
        candidate="$((12000 + ((RANDOM + $$ + attempt) % 20000)))"
        if command -v ss &>/dev/null; then
            if ! ss -H -ltn "( sport = :${candidate} )" 2>/dev/null | grep -q .; then
                echo "${candidate}"
                return 0
            fi
        else
            echo "${candidate}"
            return 0
        fi
    done
    echo "$((12000 + ($$ % 20000)))"
}

if [[ -z "${ROS_MASTER_URI:-}" ]]; then
    MASTER_PORT="$(pick_free_port)"
    export ROS_MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
else
    export ROS_MASTER_URI
fi
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"

MASTER_PORT="${ROS_MASTER_URI##*:}"
MASTER_PORT="${MASTER_PORT%%/*}"
if ! [[ "${MASTER_PORT}" =~ ^[0-9]+$ ]]; then
    MASTER_PORT=11311
fi

if [[ ! -f "${WS_ROOT}/devel/setup.bash" ]]; then
    echo "[ERROR] 未找到工作区环境: ${WS_ROOT}/devel/setup.bash"
    echo "        可通过环境变量 WS_ROOT 指定工作区根目录（例如 /home/ROS_NOETIC_WS/rm_ws）"
    exit 1
fi

source "${WS_ROOT}/devel/setup.bash"

if [[ ! -x "${BT_BIN}" ]]; then
    echo "[ERROR] 未找到可执行文件: ${BT_BIN}"
    echo "        请先构建: catkin build --no-status rm_behavior_tree"
    exit 1
fi

# ===========================================================
# 清理函数
# ===========================================================
PIDS=()
cleanup() {
    echo ""
    echo "========== 清理进程 =========="
    for p in "${PIDS[@]}"; do
        if kill -0 "$p" 2>/dev/null; then
            kill "$p" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
    if [[ -n "${TEMP_ROS_HOME}" && -d "${TEMP_ROS_HOME}" ]]; then
        rm -rf "${TEMP_ROS_HOME}"
    fi
    echo "清理完成"
}
trap cleanup EXIT INT TERM

wait_for_service() {
    local service_name="$1"
    local timeout_sec="$2"
    local deadline=$((SECONDS + timeout_sec))
    while (( SECONDS < deadline )); do
        if rosservice info "${service_name}" &>/dev/null; then
            return 0
        fi
        sleep 0.2
    done
    return 1
}

wait_for_node() {
    local node_name="$1"
    local timeout_sec="$2"
    local deadline=$((SECONDS + timeout_sec))
    while (( SECONDS < deadline )); do
        if rosnode list 2>/dev/null | grep -qx "${node_name}"; then
            return 0
        fi
        sleep 0.2
    done
    return 1
}

wait_for_file_pattern() {
    local file_path="$1"
    local pattern="$2"
    local timeout_sec="$3"
    local deadline=$((SECONDS + timeout_sec))
    while (( SECONDS < deadline )); do
        if [[ -f "${file_path}" ]] && grep -qE "${pattern}" "${file_path}" 2>/dev/null; then
            return 0
        fi
        sleep 0.2
    done
    return 1
}

wait_for_bt_ready() {
    local timeout_sec="$1"
    local deadline=$((SECONDS + timeout_sec))
    while (( SECONDS < deadline )); do
        if ! kill -0 "${BT_PID}" 2>/dev/null; then
            return 1
        fi
        if rosnode list 2>/dev/null | grep -qx "/rm_behavior_tree"; then
            if [[ -f "${BT_LOG}" ]] && grep -q "\\[BT\\]" "${BT_LOG}" 2>/dev/null; then
                return 0
            fi
            local node_info
            node_info="$(rosnode info /rm_behavior_tree 2>/dev/null || true)"
            if grep -Fq "/rm_referee/game_status" <<<"${node_info}" &&
               grep -Fq "/rm_referee/game_robot_status" <<<"${node_info}" &&
               grep -Fq "/track" <<<"${node_info}"; then
                return 0
            fi
        fi
        sleep 0.2
    done
    return 1
}

# ===========================================================
# 1. 确保 roscore
# ===========================================================
echo ">>> [1/6] 检查 roscore ..."
echo "    ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "    ROS_HOME=${ROS_HOME}"
if ! rostopic list &>/dev/null; then
    echo "    启动 roscore ..."
    roscore -p "${MASTER_PORT}" &
    PIDS+=($!)
fi
for _ in $(seq 1 20); do
    if rostopic list &>/dev/null; then
        break
    fi
    sleep 0.5
done
if ! rostopic list &>/dev/null; then
    echo "    [ERROR] roscore 未就绪"
    exit 1
fi
echo "    roscore OK"

# ===========================================================
# 2. 加载参数
# ===========================================================
echo ">>> [2/6] 加载参数 ${YAML} ..."
rosparam load "${YAML}"

case "${SCENARIO}" in
    enemy_revive_invincible_dirty_on)
        rosparam set /rm_behavior_tree/auto/enable_enemy_revive_invincible_detection true
        ;;
    enemy_revive_invincible_dirty_off)
        rosparam set /rm_behavior_tree/auto/enable_enemy_revive_invincible_detection false
        ;;
    capacitor_middle_area_demo)
        rosparam set /rm_behavior_tree/auto/enable_capacitor_strategy true
        ;;
    capacitor_middle_area_disabled)
        rosparam set /rm_behavior_tree/auto/enable_capacitor_strategy false
        ;;
esac

# 验证关键参数
if ! rosparam get /rm_behavior_tree/yaw/pid/p &>/dev/null; then
    echo "    [ERROR] yaw/pid 参数未加载成功！退出"
    exit 1
fi
echo "    参数加载完成"

# ===========================================================
# 3. 启动 fake dynamic_reconfigure + fake_tf_publisher
# ===========================================================
echo ">>> [3/6] 启动 fake_dynreconf_server + fake_tf_publisher ..."
python3 "${SIM_DIR}/fake_dynreconf_server.py" &
PIDS+=($!)
if ! wait_for_service "/move_base_flex/GlobalPlanner/set_parameters" 10; then
    echo "    [ERROR] fake_dynreconf_server 未就绪: /move_base_flex/GlobalPlanner/set_parameters"
    exit 1
fi
if ! wait_for_service "/move_base_flex/set_parameters" 10; then
    echo "    [ERROR] fake_dynreconf_server 未就绪: /move_base_flex/set_parameters"
    exit 1
fi
python3 "${SIM_DIR}/fake_tf_publisher.py" &
PIDS+=($!)
if ! wait_for_node "/fake_tf_publisher" 10; then
    echo "    [ERROR] fake_tf_publisher 未就绪"
    exit 1
fi
echo "    fake servers OK"

# ===========================================================
# 4. 启动 BT 节点 (日志输出到文件)
# ===========================================================
echo ">>> [4/6] 启动 rm_behavior_tree 节点 ..."
echo "    日志输出: ${BT_LOG}"

# 确保 enable_controller_manager=false, 使用 __name 映射节点名
ROS_NAMESPACE="" ${BT_BIN} \
    __name:=rm_behavior_tree \
    _xml_file_path:="${XML}" \
    _enable_controller_manager:=false \
    > "${BT_LOG}" 2>&1 &
PIDS+=($!)
BT_PID=${PIDS[-1]}

# 检查 BT 进程是否存活
if ! kill -0 "$BT_PID" 2>/dev/null; then
    echo "    [ERROR] BT 节点启动失败！查看日志:"
    tail -20 "${BT_LOG}"
    exit 1
fi
if ! wait_for_node "/rm_behavior_tree" 10; then
    echo "    [ERROR] BT 节点未注册到 ROS master"
    tail -20 "${BT_LOG}" || true
    exit 1
fi
if ! wait_for_bt_ready 15; then
    echo "    [ERROR] BT 节点未完成 ready 检查"
    tail -20 "${BT_LOG}" || true
    exit 1
fi
echo "    BT 节点 PID=${BT_PID}, 运行中"

# ===========================================================
# 5. 启动 match_simulator
# ===========================================================
echo ">>> [5/6] 启动 match_simulator (scenario=${SCENARIO}, speed=${SPEED}) ..."
CAP_LOG="${CAP_LOG}" python3 "${SIM_DIR}/match_simulator.py" --scenario "${SCENARIO}" --speed "${SPEED}" --external-tf \
    > "${SIM_LOG}" 2>&1 &
PIDS+=($!)
SIM_PID=${PIDS[-1]}
if ! wait_for_node "/match_simulator" 10; then
    echo "    [ERROR] match_simulator 未注册到 ROS master"
    exit 1
fi
sleep 1
if ! kill -0 "${SIM_PID}" 2>/dev/null; then
    echo "    [ERROR] match_simulator 提前退出"
    exit 1
fi
if ! wait_for_file_pattern "${BT_LOG}" "\\[BT\\]" 15; then
    echo "    [ERROR] BT 日志在仿真启动后无输出"
    tail -20 "${BT_LOG}" || true
    exit 1
fi
echo "    match_simulator 运行中"

# ===========================================================
# 6. 启动 per_second_logger (前台运行)
# ===========================================================
echo ">>> [6/6] 启动日志采集器 (前台, ${DURATION}s) ..."
echo "    监控 BT 日志文件: ${BT_LOG}"
echo ""

python3 "${SIM_DIR}/per_second_logger.py" "${BT_LOG}" ${DURATION}

if [[ -f "${CAP_LOG}" ]]; then
    cat "${CAP_LOG}" >> "${BT_LOG}"
fi

echo ""
echo "========== 测试完成 =========="
echo "完整 BT 日志: ${BT_LOG}"
echo "超电监控日志: ${CAP_LOG}"
echo "仿真驱动日志: ${SIM_LOG}"
echo "BT 日志行数: $(wc -l < "${BT_LOG}")"
