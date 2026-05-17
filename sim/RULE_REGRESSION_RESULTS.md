# RM Behavior Tree Rule Regression Results

- Date: 2026-03-07 20:26:50
- Workspace: /home/ROS_NOETIC_WS/rm_ws
- Branch: unknown

## Scenario Results

### full_match

- Result: **PASS**
- run_full_test.sh exit code: 0
- BT log: /tmp/bt_sim_output_143057.log
- Raw run output: /tmp/rm_bt_reg_full_match_7Rp2.log
- Verifier:
```text
[PASS] full_match: 所有断言通过 (2319 条 BT 记录)
```

### rulebook_full_chain

- Result: **FAIL**
- run_full_test.sh exit code: 0
- BT log: /tmp/bt_sim_output_161768.log
- Raw run output: /tmp/rm_bt_reg_rulebook_full_chain_CTtb.log
- Verifier:
```text
[FAIL] rulebook_full_chain: 2 项
  - 未覆盖到底盘模式: ABH, AMR, C, GHRA, UC
  - 比赛末段样本不足: 最晚底盘日志时间=102.00s
```

### chase_gate_020b

- Result: **PASS**
- run_full_test.sh exit code: 0
- BT log: /tmp/bt_sim_output_169702.log
- Raw run output: /tmp/rm_bt_reg_chase_gate_020b_XGRb.log
- Verifier:
```text
[PASS] chase_gate_020b: 所有断言通过 (526 条 BT 记录)
```

### referee_offline

- Result: **PASS**
- run_full_test.sh exit code: 0
- BT log: /tmp/bt_sim_output_174808.log
- Raw run output: /tmp/rm_bt_reg_referee_offline_NNGk.log
- Verifier:
```text
[PASS] referee_offline: 所有断言通过 (575 条 BT 记录)
```

### attack_phase

- Result: **PASS**
- run_full_test.sh exit code: 0
- BT log: /tmp/bt_sim_output_180359.log
- Raw run output: /tmp/rm_bt_reg_attack_phase_WEKp.log
- Verifier:
```text
[PASS] attack_phase: 所有断言通过 (573 条 BT 记录)
```

### no_bullets

- Result: **PASS**
- run_full_test.sh exit code: 0
- BT log: /tmp/bt_sim_output_185906.log
- Raw run output: /tmp/rm_bt_reg_no_bullets_LrdF.log
- Verifier:
```text
[PASS] no_bullets: 所有断言通过 (527 条 BT 记录)
```

### hp_urgent

- Result: **PASS**
- run_full_test.sh exit code: 0
- BT log: /tmp/bt_sim_output_191107.log
- Raw run output: /tmp/rm_bt_reg_hp_urgent_eQbh.log
- Verifier:
```text
[PASS] hp_urgent: 所有断言通过 (822 条 BT 记录)
```

### enemy_invincible_transition

- Result: **PASS**
- run_full_test.sh exit code: 0
- BT log: /tmp/bt_sim_output_198391.log
- Raw run output: /tmp/rm_bt_reg_enemy_invincible_transition_GulR.log
- Verifier:
```text
[PASS] enemy_invincible_transition: 所有断言通过 (464 条 BT 记录)
```

### weak_rfid_jitter

- Result: **PASS**
- run_full_test.sh exit code: 0
- BT log: /tmp/bt_sim_output_203034.log
- Raw run output: /tmp/rm_bt_reg_weak_rfid_jitter_alEC.log
- Verifier:
```text
[PASS] weak_rfid_jitter: 所有断言通过 (430 条 BT 记录)
```

### weak_localization_bias_supply

- Result: **PASS**
- run_full_test.sh exit code: 0
- BT log: /tmp/bt_sim_output_207307.log
- Raw run output: /tmp/rm_bt_reg_weak_localization_bias_supply_Pckx.log
- Verifier:
```text
[PASS] weak_localization_bias_supply: 所有断言通过 (427 条 BT 记录)
```

### conflict_chaos

- Result: **PASS**
- run_full_test.sh exit code: 0
- BT log: /tmp/bt_sim_output_211844.log
- Raw run output: /tmp/rm_bt_reg_conflict_chaos_DyT7.log
- Verifier:
```text
[PASS] conflict_chaos: 所有断言通过 (624 条 BT 记录)
```

## Summary

- Total: 11
- Pass: 10
- Fail: 1

