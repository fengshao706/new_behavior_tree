#!/bin/bash
set -euo pipefail

SIM_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_ONE="${SIM_DIR}/run_full_test.sh"
VERIFY="${SIM_DIR}/verify_bt_behavior.py"
SPEED="${1:-3.0}"
OUT_FILE="$(mktemp /tmp/rm_bt_rulebook_full_chain_XXXX.log)"
MASTER_PORT="${ROS_MASTER_PORT:-$((12000 + (RANDOM % 1000)))}"

# Use an isolated ROS master to avoid interference from pre-existing nodes/topics.
export ROS_MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"

echo ">>> 运行规则手册全链路场景: rulebook_full_chain (speed=${SPEED})"
echo ">>> 使用独立 ROS_MASTER_URI=${ROS_MASTER_URI}"
if bash "${RUN_ONE}" "rulebook_full_chain" "${SPEED}" | tee "${OUT_FILE}"; then
  RUN_STATUS=0
else
  RUN_STATUS=$?
fi

BT_LOG="$(grep -oE '完整 BT 日志: /tmp/[^ ]+' "${OUT_FILE}" | tail -1 | sed 's/^完整 BT 日志: //')"
if [[ ${RUN_STATUS} -ne 0 || -z "${BT_LOG}" || ! -f "${BT_LOG}" ]]; then
  echo "[FAIL] 运行失败，无法找到有效 BT 日志。run_status=${RUN_STATUS}, out=${OUT_FILE}"
  exit 1
fi

echo ""
echo ">>> 运行模式覆盖校验..."
python3 "${VERIFY}" "rulebook_full_chain" "${BT_LOG}"
echo "[PASS] 规则手册全链路场景通过。BT 日志: ${BT_LOG}"
