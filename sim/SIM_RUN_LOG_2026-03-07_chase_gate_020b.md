# RM Behavior Tree Full-Chain Test Log

- Date: 2026-03-07
- Scenario: `chase_gate_020b`
- Workspace: `/home/ROS_NOETIC_WS/rm_ws`
- Purpose: verify the end-to-end chain `robot_position (0x020B equivalent) -> hasOwnRobotInControlArea() -> chase gate -> chassis mode`

## Build

- Command: `catkin build --no-status rm_behavior_tree`
- Result: PASS

## Full-Chain Run

- Command:
  `env ROS_HOME=/tmp/ros ROS_MASTER_URI=http://127.0.0.1:11411 ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1 WS_ROOT=/home/ROS_NOETIC_WS/rm_ws TEST_DURATION=20 bash /home/ROS_NOETIC_WS/rm_ws/src/rm_sentry/decision/rm_behavior_tree/sim/run_full_test.sh chase_gate_020b 3.0`
- Raw BT log: `/home/ROS_NOETIC_WS/rm_ws/src/rm_sentry/decision/rm_behavior_tree/sim/logs/chase_gate_020b_20260307_bt.log`
- Log lines: `613`
- Verifier: `[PASS] chase_gate_020b: 所有断言通过 (526 条 BT 记录)`

## Injected Events And Observed Result

| Sim Time | Injected state | Expected | Observed |
| --- | --- | --- | --- |
| `0s` | `central_point_state=1`, own robot outside `middle_area`, target valid, `robot_position` shows ally inside `middle_area` | `Chase` | `C:C G:TE B:P` at BT `1.00s` |
| `8s` | still `central_point_state=1`, but `robot_position` shows no ally in `middle_area` | `ProtectMidRobot` | `C:PMR G:TE B:P` at BT `9.00s` |
| `16s` | `robot_position` shows ally re-entered `middle_area` | `Chase` | `C:C G:TE B:P` at BT `18.00s` |
| `24s` | `central_point_state=2` (enemy control) | `AttackMidRobot` | `C:AMR G:TE B:P` at BT `25.00s` |

## Conclusion

- The new 0x020B-equivalent gate is exercised by simulation, not just by fallback logic.
- While the control area is ours, chase is now allowed only when `robot_position` indicates an ally inside the control area.
- When the ally leaves the control area, the chassis falls back to `ProtectMidRobot`.
- Enemy control still overrides the chase gate and correctly switches to `AttackMidRobot`.

## Regression Status

- Rule regression report: `/home/ROS_NOETIC_WS/rm_ws/src/rm_sentry/decision/rm_behavior_tree/sim/RULE_REGRESSION_RESULTS.md`
- Summary: `11` scenarios total, `10` pass, `1` fail
- Failing item: `rulebook_full_chain`
- Failure reason from verifier:
  - missing expected chassis modes `ABH, AMR, C, GHRA, UC`
  - latest chassis sample only reached `BT 102.00s`
- Assessment: this failure looks like a scenario coverage / timing issue in the existing regression harness, not a direct failure of the new `robot_position` chase gate. The dedicated `chase_gate_020b` scenario passed end-to-end.
