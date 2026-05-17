#!/bin/bash
set -u

SIM_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_ONE="${SIM_DIR}/run_full_test.sh"
VERIFY="${SIM_DIR}/verify_bt_behavior.py"
REPORT_STEM="RULE_REGRESSION_RESULTS_$(date '+%Y%m%d_%H%M%S')_$$"
REPORT="${SIM_DIR}/${REPORT_STEM}.md"
LATEST_REPORT="${SIM_DIR}/RULE_REGRESSION_RESULTS_latest.md"
WS_ROOT="${WS_ROOT:-$(cd "${SIM_DIR}/../../../../.." && pwd)}"
SENTRY_REPO="${SENTRY_REPO:-${WS_ROOT}/src/rm_sentry}"

SCENARIOS=(
  "full_match"
  "rulebook_full_chain"
  "rulebook_full_chain_dirty"
  "chase_gate_020b"
  "control_area_dirty_gate"
  "posture_transitions"
  "capacitor_middle_area_demo"
  "capacitor_middle_area_disabled"
  "referee_offline"
  "attack_phase"
  "no_bullets"
  "hp_urgent"
  "enemy_invincible_transition"
  "enemy_revive_invincible_dirty_on"
  "enemy_revive_invincible_dirty_off"
  "weak_rfid_jitter"
  "weak_localization_bias_supply"
  "conflict_chaos"
)

pass_count=0
fail_count=0
total_count=0

{
  echo "# RM Behavior Tree Rule Regression Results"
  echo ""
  echo "- Date: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "- Workspace: ${WS_ROOT}"
  echo "- Branch: $(git -C "${SENTRY_REPO}" branch --show-current 2>/dev/null || echo unknown)"
  echo ""
  echo "## Scenario Results"
  echo ""
} > "${REPORT}"

for scenario in "${SCENARIOS[@]}"; do
  total_count=$((total_count + 1))
  out_file="$(mktemp /tmp/rm_bt_reg_${scenario}_XXXX.log)"

  echo ""
  echo "========== Running scenario: ${scenario} =========="
  if bash "${RUN_ONE}" "${scenario}" > "${out_file}" 2>&1; then
    run_status=0
  else
    run_status=$?
  fi

  bt_log="$(grep -oE '完整 BT 日志: /tmp/[^ ]+' "${out_file}" | tail -1 | sed 's/^完整 BT 日志: //')"
  verify_status=99
  verify_summary=""

  if [[ ${run_status} -eq 0 && -n "${bt_log}" && -f "${bt_log}" ]]; then
    verify_out="$(python3 "${VERIFY}" "${scenario}" "${bt_log}" 2>&1)"
    verify_status=$?
    verify_summary="$(echo "${verify_out}" | tail -n +1)"
  else
    verify_summary="未获得有效 BT 日志，run_full_test 退出码=${run_status}"
    verify_status=1
  fi

  if [[ ${run_status} -eq 0 && ${verify_status} -eq 0 ]]; then
    result="PASS"
    pass_count=$((pass_count + 1))
  else
    result="FAIL"
    fail_count=$((fail_count + 1))
  fi

  echo "${result}: ${scenario}"
  echo "  output log: ${out_file}"
  if [[ -n "${bt_log}" ]]; then
    echo "  bt log: ${bt_log}"
  fi
  echo "${verify_summary}" | sed 's/^/  verify: /'

  {
    echo "### ${scenario}"
    echo ""
    echo "- Result: **${result}**"
    echo "- run_full_test.sh exit code: ${run_status}"
    echo "- BT log: ${bt_log:-N/A}"
    echo "- Raw run output: ${out_file}"
    echo "- Verifier:"
    echo '```text'
    echo "${verify_summary}"
    echo '```'
    echo ""
  } >> "${REPORT}"
done

{
  echo "## Summary"
  echo ""
  echo "- Total: ${total_count}"
  echo "- Pass: ${pass_count}"
  echo "- Fail: ${fail_count}"
  echo ""
} >> "${REPORT}"

echo ""
echo "Regression report written to: ${REPORT}"
ln -sfn "$(basename "${REPORT}")" "${LATEST_REPORT}"
echo "Latest report link: ${LATEST_REPORT}"
if [[ ${fail_count} -gt 0 ]]; then
  exit 1
fi
exit 0
