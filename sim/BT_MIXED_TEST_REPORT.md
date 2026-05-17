# rm_behavior_tree 多情况混合测试报告

- 生成时间: 2026-03-05
- 工作区: `/home/ROS_NOETIC_WS/rm_ws`
- 结果来源: `sim/run_rule_regression.sh` + `sim/verify_bt_behavior.py`

## 1. 结论

有，已尝试并通过多种“混合情况”测试。  
本次共执行 10 个场景，`PASS=10`，`FAIL=0`。

## 2. 是否覆盖“混合情况”

已覆盖，重点包括以下混合场景:

1. `conflict_chaos`  
组合了: 敌方补给区无敌、裁判断联、低血、无弹、死亡复活、RFID 抖动、补给解除弱态等连续冲突。
2. `full_match`  
完整 5 分钟流程，跨阶段混合了中点攻防、追踪丢失、血量告急、补给、死亡复活、裁判离线/恢复等。
3. `rulebook_full_chain`  
按规则链路混合验证底盘模式全覆盖 (`GMA/PMR/C/UC/AMR/GHRA/ABH/CSG/AS`)。

对应场景定义可见:
- `build_full_match_events`: `sim/match_simulator.py` 150 行附近
- `build_rulebook_full_chain_events`: `sim/match_simulator.py` 326 行附近
- `build_conflict_chaos_events`: `sim/match_simulator.py` 751 行附近

## 3. 回归结果总表

| 场景 | 类型 | 结果 | BT 记录数 | BT 日志 |
|---|---|---|---:|---|
| full_match | 多因素混合 | PASS | 2775 | `/tmp/bt_sim_output_277074.log` |
| rulebook_full_chain | 规则全链路混合 | PASS | 2831 | `/tmp/bt_sim_output_300703.log` |
| referee_offline | 单主题（离线恢复） | PASS | 578 | `/tmp/bt_sim_output_325242.log` |
| attack_phase | 单主题（攻击持续） | PASS | 581 | `/tmp/bt_sim_output_330996.log` |
| no_bullets | 单主题（弹药约束） | PASS | 526 | `/tmp/bt_sim_output_336695.log` |
| hp_urgent | 单主题（血量紧急） | PASS | 796 | `/tmp/bt_sim_output_342022.log` |
| enemy_invincible_transition | 双因素（无敌区切换） | PASS | 474 | `/tmp/bt_sim_output_349494.log` |
| weak_rfid_jitter | 双因素（复活弱态+RFID抖动） | PASS | 436 | `/tmp/bt_sim_output_354191.log` |
| weak_localization_bias_supply | 双因素（定位偏差+补给） | PASS | 437 | `/tmp/bt_sim_output_358529.log` |
| conflict_chaos | 多因素强冲突混合 | PASS | 627 | `/tmp/bt_sim_output_363162.log` |

## 4. 混合场景重点断言结果

### 4.1 `conflict_chaos` (多冲突)

验证通过项（摘要）:

1. 敌方在补给区无敌阶段不允许 `Push`。
2. 离开无敌区后恢复 `Push`。
3. 低血阶段出现 `GHRA`。
4. 裁判离线阶段出现 `ABH` 且射手 `Stop`。
5. 无弹阶段不 `Push`，补弹后恢复 `Push`。
6. 死亡时 `ReviveIfDead=dead`，复活弱态期间射手维持 `Ready`，补给稳定后恢复 `Push`。

### 4.2 `full_match` (完整流程混合)

验证通过项（摘要）:

1. 210s 死亡触发 `ReviveIfDead FAILURE(revive=dead)`。
2. 复活后出现校准窗口 `revive=calibrating`。
3. 复活弱态期间底盘 `GHRA`、射手 `Ready`。
4. 进入/离开补给区后，模式按预期切换。

### 4.3 `rulebook_full_chain` (模式全链路混合)

验证通过项（摘要）:

1. 底盘模式覆盖齐全: `GMA/PMR/C/UC/AMR/GHRA/ABH/CSG/AS`。
2. 场景起止时间覆盖比赛前后关键时段，模式切换链路连续。

## 5. 执行命令（本次）

```bash
catkin build --no-status rm_behavior_tree
ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1 \
bash sim/run_rule_regression.sh
```

## 6. 产物文件

1. 回归明细: `sim/RULE_REGRESSION_RESULTS.md`
2. 本报告: `sim/BT_MIXED_TEST_REPORT.md`

