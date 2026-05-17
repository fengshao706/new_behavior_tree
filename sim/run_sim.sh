#!/bin/bash
# =============================================================================
# run_sim.sh  —  rm_behavior_tree RMUC 2026 比赛仿真一键启动脚本
# =============================================================================
# 用法：
#   ./run_sim.sh                        # 默认完整比赛场景，加速3x
#   ./run_sim.sh --scenario hp_urgent   # 血量告急专项场景
#   ./run_sim.sh --scenario posture_transitions  # posture 切换专项场景
#   ./run_sim.sh --speed 1.0            # 实时速度（不加速）
#   ./run_sim.sh --rviz                 # 同时打开 rviz
#   ./run_sim.sh --no-bt                # 只运行仿真驱动，不启动行为树节点
#
# 依赖检查：
#   - ROS 环境已 source：source ~/ros_ws/rm_ws/devel/setup.bash
#   - rm_behavior_tree 已编译：catkin build rm_behavior_tree
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../../../../.." && pwd)"  # 指向 rm_ws 根目录

# ─── 颜色 ─────────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ─── 默认参数 ─────────────────────────────────────────────────────────────────
SCENARIO="full_match"
SPEED="3.0"
RVIZ="false"
LAUNCH_BT="true"
LOG_FILE="/tmp/bt_sim_$(date +%Y%m%d_%H%M%S).txt"

# ─── 参数解析 ─────────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario) SCENARIO="$2"; shift 2 ;;
    --speed)    SPEED="$2";    shift 2 ;;
    --rviz)     RVIZ="true";   shift ;;
    --no-bt)    LAUNCH_BT="false"; shift ;;
    --log)      LOG_FILE="$2"; shift 2 ;;
    -h|--help)
      echo "用法: $0 [--scenario <名称>] [--speed <倍率>] [--rviz] [--no-bt] [--log <文件>]"
      echo "  场景: full_match rulebook_full_chain chase_gate_020b posture_transitions capacitor_middle_area_demo capacitor_middle_area_disabled hp_urgent death_revive attack_phase"
      exit 0 ;;
    *) echo -e "${RED}未知参数: $1${NC}"; exit 1 ;;
  esac
done

echo -e "${BOLD}${CYAN}"
echo "═══════════════════════════════════════════════════"
echo "  rm_behavior_tree RMUC 2026 比赛仿真"
echo "  场景: $SCENARIO   加速: ${SPEED}x   RViz: $RVIZ"
echo "═══════════════════════════════════════════════════"
echo -e "${NC}"

# ─── 检查 ROS 环境 ─────────────────────────────────────────────────────────────
if ! command -v rosrun &> /dev/null; then
  echo -e "${RED}[错误] 未找到 rosrun，请先执行:${NC}"
  echo "  source ${WS_ROOT}/devel/setup.bash"
  exit 1
fi

# ─── 检查 roscore ──────────────────────────────────────────────────────────────
if ! rostopic list &> /dev/null; then
  echo -e "${YELLOW}[信息] roscore 未运行，在后台启动...${NC}"
  roscore &
  ROSCORE_PID=$!
  sleep 2
  echo -e "${GREEN}[OK] roscore 已启动 (PID=$ROSCORE_PID)${NC}"
fi

# ─── 清理函数（Ctrl+C 时清理子进程）──────────────────────────────────────────
PIDS=()
cleanup() {
  echo -e "\n${YELLOW}[Simulator] 正在关闭所有仿真节点...${NC}"
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  if [[ -n "${ROSCORE_PID:-}" ]]; then
    kill "$ROSCORE_PID" 2>/dev/null || true
  fi
  echo -e "${GREEN}[OK] 仿真已停止。日志文件: ${LOG_FILE}${NC}"
  exit 0
}
trap cleanup SIGINT SIGTERM

# ─── 启动 TF 仿真发布器 ────────────────────────────────────────────────────────
echo -e "${CYAN}[1/4] 启动 TF 仿真发布器...${NC}"
python3 "${SCRIPT_DIR}/fake_tf_publisher.py" &
PIDS+=($!)
sleep 1

# ─── 启动行为树节点（可选）────────────────────────────────────────────────────
if [[ "$LAUNCH_BT" == "true" ]]; then
  echo -e "${CYAN}[2/4] 启动 rm_behavior_tree 决策节点...${NC}"
  # 加载参数
  rosparam load "${SCRIPT_DIR}/../test/sentry.yaml" /rm_behavior_tree 2>/dev/null || true
  # 启动节点
  rosrun rm_behavior_tree rm_behavior_tree \
    _xml_file_path:="${SCRIPT_DIR}/../config/main_tree.xml" \
    _enable_controller_manager:=false &
  PIDS+=($!)
  sleep 1
else
  echo -e "${YELLOW}[2/4] 跳过行为树节点（--no-bt）${NC}"
fi

# ─── 启动监控器 ────────────────────────────────────────────────────────────────
echo -e "${CYAN}[3/4] 启动行为树状态监控器... (日志 → ${LOG_FILE})${NC}"
python3 "${SCRIPT_DIR}/bt_monitor.py" --output "${LOG_FILE}" &
PIDS+=($!)
sleep 0.5

# ─── 启动比赛场景驱动 ──────────────────────────────────────────────────────────
echo -e "${CYAN}[4/4] 启动比赛场景仿真驱动 (场景=${SCENARIO}, 加速=${SPEED}x)...${NC}"
echo ""
python3 "${SCRIPT_DIR}/match_simulator.py" \
  --scenario "${SCENARIO}" \
  --speed    "${SPEED}"
PIDS+=($!)

# ─── 等待结束 ─────────────────────────────────────────────────────────────────
wait "${PIDS[-1]}"
cleanup
