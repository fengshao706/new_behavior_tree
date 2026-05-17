# rm_behavior_tree Simulation Run Log

## Run Summary

- Date: 2026-03-07 UTC
- Scenario: `full_match`
- Package: `rm_ws/src/rm_sentry/decision/rm_behavior_tree`
- Command:
  `env ROS_HOME=/tmp/ros ROS_MASTER_URI=http://127.0.0.1:11411 ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1 WS_ROOT=/home/ROS_NOETIC_WS/rm_ws TEST_DURATION=120 bash sim/run_full_test.sh full_match 3.0`
- Result: simulation completed, exit code `0`
- Raw BT log lines: `3351`
- Raw BT log copy: `sim/logs/full_match_20260307_bt.log`

## Injected Scenario Events

These events come from `sim/match_simulator.py::build_full_match_events()` and were injected during this run.

| t (s) | Event | Injected change |
| --- | --- | --- |
| 0 | `比赛开始` | `game_progress = IN_BATTLE` |
| 5 | `进入中间区域` | Robot pose moved to `middle_area` |
| 11 | `小陀螺启动` | Time-based gyro activation cue |
| 25 | `我方占领中心点` | `central_point_state = 1` |
| 40 | `发现跟踪目标` | `track_id = 3`, begin target tracking |
| 55 | `敌方占领中心点` | `central_point_state = 2` |
| 70 | `跟踪丢失` | `track_id = 0` |
| 80 | `血量告急` | `remain_hp = 250` |
| 90 | `到达补给区` | Enter supply area, RFID on |
| 100 | `血量回满` | `remain_hp = 400`, leave supply area |
| 110 | `双方占领中心点` | `central_point_state = 3` |
| 130 | `重见目标` | `track_id = 1` |
| 150 | `保守限制期` | Enter conservative chase restriction period |
| 180 | `弹药耗尽` | `bullet_17mm = 0` |
| 190 | `补弹完成` | `bullet_17mm = 200` |
| 210 | `机器人死亡` | `remain_hp = 0` |
| 218 | `机器人复活` | `remain_hp = 80` |
| 224 | `复活后进入补给区` | Enter supply area after revive |
| 228 | `复活后离开补给区` | Leave supply area, `remain_hp = 220` |
| 230 | `裁判系统断联` | `referee_online = false` |
| 240 | `裁判系统恢复` | `referee_online = true` |
| 260 | `终局血量告急` | `remain_hp = 100` |
| 280 | `比赛即将结束` | `remain_hp = 400` |
| 300 | `比赛结束` | `game_progress = CALCULATION` |

## Observed BT Mode Timeline

The following transitions were extracted from `GetChassisDecisions`, `GetGimbalDecisions`, and `GetShooterDecisions` in the raw BT log.

| BT time (s) | Chassis | Gimbal | Booster | Area | Note |
| --- | --- | --- | --- | --- | --- |
| 0 | `AS -> CSG` | `AS -> YSR` | `S -> R` | `own_protect_mid_robot_area` | Initial offline-safe mode immediately settles into pre-battle slow gyro |
| 1 | `CSG -> GMA` | `YSR -> RSE` | `R` | `own_protect_mid_robot_area` | Match enters battle and starts moving/searching |
| 26 | `GMA -> PMR` | `RSE` | `R` | `own_protect_mid_robot_area` | Responds to our side holding center point |
| 41 | `PMR -> C` | `RSE -> TE` | `R -> P` | `own_protect_mid_robot_area` | Tracking target appears, chase and push enabled |
| 56 | `C -> AMR` | `TE` | `P` | `own_protect_mid_robot_area` | Enemy center-point control pushes bot into attack-mid mode |
| 71 | `AMR` | `TE -> FSE` | `P -> R` | `own_protect_mid_robot_area` | Track lost during `AttackMidRobot`, enters fan search |
| 86 | `AMR` | `FSE -> RSE` | `R` | `own_protect_mid_robot_area` | Fan-search retention ends, back to round search |
| 132 | `AMR` | `RSE -> TE` | `R -> P` | `own_leisure_area` | Target reacquired in rear leisure area |
| 182 | `AMR` | `TE` | `P -> R` | `own_leisure_area` | Firing stops after ammo depletion |
| 191 | `AMR` | `TE` | `R -> P` | `own_leisure_area` | Firing resumes after ammo refill |
| 221 | `AMR -> GHRA` | `TE -> RSE` | `P -> R` | `own_leisure_area` | Low HP triggers return-for-heal behavior |
| 230 | `GHRA -> ABH` | `RSE -> AS` | `R -> S` | `own_leisure_area` | Referee offline fallback path activates |
| 241 | `ABH -> AMR` | `AS -> FSE` | `S -> R` | `own_leisure_area` | Referee recovers and normal decision flow returns |
| 271 | `AMR` | `FSE -> RSE` | `R` | `own_leisure_area` | Fan-search window ends again |
| 300 | `AMR -> CSG` | `RSE -> YSR` | `R` | `own_leisure_area` | Match ends and behavior returns to non-battle state |

## Failure Events

Observed BT failures were concentrated in the death/revive window:

| BT time (s) | Node | Extra |
| --- | --- | --- |
| 211 | `ReviveIfDead` | `revive=dead` |
| 211 | `ReviveIfDead` | `revive=dead` |
| 211 | `ReviveIfDead` | `revive=dead` |
| 211 | `ReviveIfDead` | `revive=dead` |
| 211 | `ReviveIfDead` | `revive=dead` |
| 211 | `ReviveIfDead` | `revive=calibrating` |

Interpretation:

- The simulated death at 210 s was observed by the BT at 211 s.
- `ReviveIfDead` blocked downstream action nodes during the death/calibration window as designed.

## Warning Summary

Counts below were extracted from the raw BT log:

| Warning | Count | Comment |
| --- | --- | --- |
| `action not connect` | 183 | Planner/action interface is not fully connected in the simulator |
| missing `yaw` source frame | 139 | TF lookup asks for a `yaw` frame that does not exist in this simulation |
| missing `yaw` target frame | 139 | Same root cause as above |
| `Controller manager disabled; skipping controller setup.` | 1 | Expected because `_enable_controller_manager=false` was used |

These warnings did not stop the simulation from reaching `Time:300.00`, but they indicate the current simulator is behavior-centric and not a full controller/planner integration environment.

## Verification Note

Running

`python3 sim/verify_bt_behavior.py full_match sim/logs/full_match_20260307_bt.log`

returned one failure:

- `缺少证据: 210s 死亡后 ReviveIfDead FAILURE(revive=dead)`

The raw log shows the first matching failure at `211.0 s`, so this looks like a strict timing mismatch between the verifier expectation and the actual BT observation boundary, not a missing death-handling path.

## Additional Notes

- `InverseGimbal` was not observed in this `full_match` scenario.
- The simulation required loopback ROS networking to run reliably in this environment:
  `ROS_MASTER_URI=http://127.0.0.1:11411`, `ROS_IP=127.0.0.1`, `ROS_HOSTNAME=127.0.0.1`.
