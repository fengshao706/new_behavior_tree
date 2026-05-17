#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_bt_behavior.py

解析 rm_behavior_tree 的 BT stdout 日志，对关键规则行为做断言校验。

用法:
  python3 verify_bt_behavior.py <scenario> <bt_log_file>
"""

import re
import sys
from dataclasses import dataclass
from typing import List, Callable, Tuple


LOG_RE = re.compile(
    r"\[BT\] (?P<node>\S+)\s+(?P<status>SUCCESS|FAILURE|RUNNING)"
    r"\s+C:(?P<chassis>\S+)\s+G:(?P<gimbal>\S+)\s+B:(?P<booster>\S+)"
    r"\s+pos=\((?P<px>[\-\d.]+),(?P<py>[\-\d.]+)\)(?:@(?P<area>\S+))?"
    r"\s+Enemy:(?P<enemy>\S+)"
    r"\s+Time:(?P<time>[\d.]+)"
    r"(?:\s+(?P<extra>.*))?"
)
CAP_RE = re.compile(r"\[CAP\] ManualToReferee time=(?P<time>[\d.]+) state=(?P<state>\d+)")
CAP_STATE_RE = re.compile(r"cap_state=(?P<state>\d+)")

POWER_LIMIT_BURST = 1
POWER_LIMIT_NORMAL = 2


@dataclass
class Rec:
    node: str
    status: str
    chassis: str
    gimbal: str
    booster: str
    area: str
    enemy: str
    bt_time: float
    extra: str


@dataclass
class CapRec:
    sim_time: float
    state: int


def parse_log(path: str) -> List[Rec]:
    out: List[Rec] = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = LOG_RE.search(line)
            if not m:
                continue
            out.append(
                Rec(
                    node=m.group("node"),
                    status=m.group("status"),
                    chassis=m.group("chassis"),
                    gimbal=m.group("gimbal"),
                    booster=m.group("booster"),
                    area=m.group("area") or "",
                    enemy=m.group("enemy"),
                    bt_time=float(m.group("time")),
                    extra=(m.group("extra") or "").strip(),
                )
            )
    return out


def parse_cap_log(path: str) -> List[CapRec]:
    out: List[CapRec] = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = CAP_RE.search(line)
            if not m:
                continue
            out.append(CapRec(sim_time=float(m.group("time")), state=int(m.group("state"))))
    return out


def filt(records: List[Rec], pred: Callable[[Rec], bool]) -> List[Rec]:
    return [r for r in records if pred(r)]


def in_time(t0: float, t1: float) -> Callable[[Rec], bool]:
    return lambda r: t0 <= r.bt_time <= t1


def any_rec(records: List[Rec], desc: str, pred: Callable[[Rec], bool], failures: List[str]):
    if not any(pred(r) for r in records):
        failures.append(f"缺少证据: {desc}")


def no_rec(records: List[Rec], desc: str, pred: Callable[[Rec], bool], failures: List[str]):
    bad = [r for r in records if pred(r)]
    if bad:
        sample = bad[0]
        failures.append(
            f"出现违规: {desc} (node={sample.node} C={sample.chassis} G={sample.gimbal} "
            f"B={sample.booster} area={sample.area} t={sample.bt_time:.2f} extra={sample.extra})"
        )


def cap_state_from_record(rec: Rec) -> int:
    m = CAP_STATE_RE.search(rec.extra)
    return int(m.group("state")) if m else -1


def any_cap_rec(records: List[CapRec], desc: str, pred: Callable[[CapRec], bool], failures: List[str]):
    if not any(pred(r) for r in records):
        failures.append(f"缺少证据: {desc}")


def no_cap_rec(records: List[CapRec], desc: str, pred: Callable[[CapRec], bool], failures: List[str]):
    bad = [r for r in records if pred(r)]
    if bad:
        sample = bad[0]
        failures.append(f"出现违规: {desc} (time={sample.sim_time:.2f} state={sample.state})")


def check_capacitor_middle_area_demo(records: List[Rec], cap_records: List[CapRec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "GMA 未到达窗口底盘超电应为 BURST",
            lambda r: r.node == "SendChassisCommand" and 1.0 <= r.bt_time <= 35.5 and
            cap_state_from_record(r) == POWER_LIMIT_BURST, f)
    any_rec(records, "middle_area 无目标窗口应回 NORMAL",
            lambda r: r.node == "SendChassisCommand" and 37.0 <= r.bt_time <= 53.5 and
            cap_state_from_record(r) == POWER_LIMIT_NORMAL, f)
    any_rec(records, "middle_area TrackEnemy 窗口应切 BURST",
            lambda r: r.node == "SendChassisCommand" and 55.0 <= r.bt_time <= 71.5 and
            cap_state_from_record(r) == POWER_LIMIT_BURST, f)
    any_rec(records, "middle_area 丢目标窗口应回 NORMAL",
            lambda r: r.node == "SendChassisCommand" and 73.0 <= r.bt_time <= 89.5 and
            cap_state_from_record(r) == POWER_LIMIT_NORMAL, f)
    any_rec(records, "离开 middle_area 后继续 GMA 应恢复 BURST",
            lambda r: r.node == "SendChassisCommand" and 91.0 <= r.bt_time <= 107.5 and
            cap_state_from_record(r) == POWER_LIMIT_BURST, f)

    any_cap_rec(cap_records, "/manual_to_referee 在 GMA 未到达窗口发布 BURST",
                lambda r: 1.0 <= r.sim_time <= 35.5 and r.state == POWER_LIMIT_BURST, f)
    any_cap_rec(cap_records, "/manual_to_referee 在 middle_area 无目标窗口发布 NORMAL",
                lambda r: 37.0 <= r.sim_time <= 53.5 and r.state == POWER_LIMIT_NORMAL, f)
    any_cap_rec(cap_records, "/manual_to_referee 在 TrackEnemy 窗口发布 BURST",
                lambda r: 55.0 <= r.sim_time <= 71.5 and r.state == POWER_LIMIT_BURST, f)
    any_cap_rec(cap_records, "/manual_to_referee 在丢目标窗口发布 NORMAL",
                lambda r: 73.0 <= r.sim_time <= 89.5 and r.state == POWER_LIMIT_NORMAL, f)
    any_cap_rec(cap_records, "/manual_to_referee 在离开 middle_area 后恢复 BURST",
                lambda r: 91.0 <= r.sim_time <= 107.5 and r.state == POWER_LIMIT_BURST, f)
    return f


def check_capacitor_middle_area_disabled(records: List[Rec], cap_records: List[CapRec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "关闭总开关后仍有底盘发送样本",
            lambda r: r.node == "SendChassisCommand" and 1.0 <= r.bt_time <= 107.5, f)
    no_rec(records, "关闭总开关后不应出现 BURST",
           lambda r: r.node == "SendChassisCommand" and 1.0 <= r.bt_time <= 107.5 and
           cap_state_from_record(r) == POWER_LIMIT_BURST, f)
    any_rec(records, "关闭总开关后底盘超电状态保持 NORMAL",
            lambda r: r.node == "SendChassisCommand" and 1.0 <= r.bt_time <= 107.5 and
            cap_state_from_record(r) == POWER_LIMIT_NORMAL, f)

    any_cap_rec(cap_records, "关闭总开关后 /manual_to_referee 仍有发布",
                lambda r: 1.0 <= r.sim_time <= 107.5, f)
    no_cap_rec(cap_records, "关闭总开关后 /manual_to_referee 不应发布 BURST",
               lambda r: 1.0 <= r.sim_time <= 107.5 and r.state == POWER_LIMIT_BURST, f)
    any_cap_rec(cap_records, "关闭总开关后 /manual_to_referee 保持 NORMAL",
                lambda r: 1.0 <= r.sim_time <= 107.5 and r.state == POWER_LIMIT_NORMAL, f)
    return f


def check_full_match(records: List[Rec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "210s 死亡后 ReviveIfDead FAILURE(revive=dead)",
            lambda r: r.node == "ReviveIfDead" and r.status == "FAILURE" and abs(r.bt_time - 210.0) < 0.6 and "revive=dead" in r.extra, f)
    any_rec(records, "复活后校准阶段 ReviveIfDead FAILURE(revive=calibrating)",
            lambda r: r.node == "ReviveIfDead" and r.status == "FAILURE" and "revive=calibrating" in r.extra, f)
    any_rec(records, "复活后弱态底盘 GHRA",
            lambda r: r.node == "GetChassisDecisions" and 220.0 <= r.bt_time <= 224.5 and r.chassis == "GHRA", f)
    any_rec(records, "复活后弱态射手 Ready",
            lambda r: r.node == "GetShooterDecisions" and 220.0 <= r.bt_time <= 224.5 and r.booster == "R", f)
    any_rec(records, "224后进入补给区 own_supply_area",
            lambda r: 224.5 <= r.bt_time <= 226.5 and r.area == "own_supply_area", f)
    any_rec(records, "228后离开补给区回 own_leisure_area",
            lambda r: 228.5 <= r.bt_time <= 231.0 and r.area == "own_leisure_area", f)
    return f


def check_enemy_invincible_transition(records: List[Rec]) -> List[str]:
    f: List[str] = []
    pre = filt(records, lambda r: 2.0 <= r.bt_time <= 9.8)
    post = filt(records, lambda r: 10.5 <= r.bt_time <= 17.5)

    any_rec(pre, "敌方在补给区时存在射手决策样本", lambda r: r.node == "GetShooterDecisions", f)
    no_rec(pre, "敌方在补给区时不应 Push",
           lambda r: r.node == "GetShooterDecisions" and r.booster == "P", f)
    any_rec(pre, "敌方在补给区时射手保持 Ready",
            lambda r: r.node == "GetShooterDecisions" and r.booster == "R", f)

    any_rec(post, "目标离开补给区后云台恢复 TrackEnemy",
            lambda r: r.node == "GetGimbalDecisions" and r.gimbal == "TE", f)
    any_rec(post, "目标离开补给区后射手恢复 Push",
            lambda r: r.node == "GetShooterDecisions" and r.booster == "P", f)
    return f


def check_weak_rfid_jitter(records: List[Rec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "死亡后 ReviveIfDead FAILURE(revive=dead)",
            lambda r: r.node == "ReviveIfDead" and r.status == "FAILURE" and "revive=dead" in r.extra, f)
    any_rec(records, "复活后弱态期间 GHRA",
            lambda r: r.node == "GetChassisDecisions" and 14.0 <= r.bt_time <= 21.5 and r.chassis == "GHRA", f)
    # 期望：RFID 抖动仅单次误触发时，弱态不应被提前解除，因此射手不应进入 Push
    no_rec(records, "复活弱态 + RFID抖动期间不应出现 Push（否则说明弱态被误解除）",
           lambda r: r.node == "GetShooterDecisions" and 14.0 <= r.bt_time <= 21.5 and r.booster == "P", f)
    any_rec(records, "复活弱态 + RFID抖动期间射手存在 Ready 样本",
            lambda r: r.node == "GetShooterDecisions" and 14.0 <= r.bt_time <= 21.5 and r.booster == "R", f)
    return f


def check_weak_localization_bias_supply(records: List[Rec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "复活后弱态期间 GHRA",
            lambda r: r.node == "GetChassisDecisions" and 14.0 <= r.bt_time <= 18.5 and r.chassis == "GHRA", f)
    any_rec(records, "定位偏差条件下弱态期间射手 Ready",
            lambda r: r.node == "GetShooterDecisions" and 14.0 <= r.bt_time <= 18.5 and r.booster == "R", f)
    any_rec(records, "定位偏差 + RFID稳定/HP增长后恢复 Push",
            lambda r: r.node == "GetShooterDecisions" and 19.0 <= r.bt_time <= 23.5 and r.booster == "P", f)
    return f


def check_referee_offline(records: List[Rec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "第一次断联期间底盘进入 ABH",
            lambda r: r.node == "GetChassisDecisions" and 3.0 <= r.bt_time <= 9.8 and r.chassis == "ABH", f)
    any_rec(records, "第一次断联期间射手进入 Stop",
            lambda r: r.node == "GetShooterDecisions" and 3.0 <= r.bt_time <= 9.8 and r.booster == "S", f)
    any_rec(records, "第一次恢复后射手可恢复 Push",
            lambda r: r.node == "GetShooterDecisions" and 10.0 <= r.bt_time <= 16.0 and r.booster == "P", f)
    any_rec(records, "第二次断联期间再次进入 Stop",
            lambda r: r.node == "GetShooterDecisions" and 16.0 <= r.bt_time <= 22.0 and r.booster == "S", f)
    any_rec(records, "第二次恢复后再次出现非Stop射击决策",
            lambda r: r.node == "GetShooterDecisions" and 22.0 <= r.bt_time <= 30.0 and r.booster in {"R", "P"}, f)
    return f


def check_chase_gate_020b(records: List[Rec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "控制区被我方占领且0x020B显示友方在控制区时进入 Chase",
            lambda r: r.node == "GetChassisDecisions" and 1.0 <= r.bt_time <= 7.5 and r.chassis == "C", f)
    any_rec(records, "首段追击窗口存在 TrackEnemy",
            lambda r: r.node == "GetGimbalDecisions" and 1.0 <= r.bt_time <= 7.5 and r.gimbal == "TE", f)

    any_rec(records, "控制区被我方占领但0x020B显示控制区无人时回 ProtectMidRobot",
            lambda r: r.node == "GetChassisDecisions" and 9.0 <= r.bt_time <= 15.5 and r.chassis == "PMR", f)
    any_rec(records, "控制区无人阻断窗口云台仍保持 TrackEnemy，说明阻断来自0x020B门槛",
            lambda r: r.node == "GetGimbalDecisions" and 9.0 <= r.bt_time <= 15.5 and r.gimbal == "TE", f)
    no_rec(records, "控制区无人阻断窗口不应继续 Chase/UnChase",
           lambda r: r.node == "GetChassisDecisions" and 9.0 <= r.bt_time <= 15.5 and r.chassis in {"C", "UC"}, f)

    any_rec(records, "友方重新进入控制区后恢复 Chase",
            lambda r: r.node == "GetChassisDecisions" and 17.0 <= r.bt_time <= 23.5 and r.chassis == "C", f)
    any_rec(records, "敌方占点后即使0x020B仍显示友方在区内也进入 AMR",
            lambda r: r.node == "GetChassisDecisions" and 25.0 <= r.bt_time <= 29.5 and r.chassis == "AMR", f)
    return f


def check_posture_transitions(records: List[Rec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "开局在 middle_area 外时底盘进入 GMA",
            lambda r: r.node == "GetChassisDecisions" and 1.0 <= r.bt_time <= 35.5 and r.chassis == "GMA", f)
    any_rec(records, "开局在 middle_area 外时 posture=3(rule=goto_middle_area)",
            lambda r: r.node == "GetSentryCmd" and 16.0 <= r.bt_time <= 35.5 and
            "mode=3" in r.extra and "rule=goto_middle_area" in r.extra, f)
    any_rec(records, "开局移动姿态对应底盘功率倍率 1.5x",
            lambda r: r.node == "SendChassisCommand" and 16.0 <= r.bt_time <= 35.5 and
            "power_limit=30" in r.extra and "posture_scale=1.500000" in r.extra, f)

    any_rec(records, "进入 middle_area 后 posture=2(rule=middle_area)",
            lambda r: r.node == "GetSentryCmd" and 37.0 <= r.bt_time <= 53.5 and
            "mode=2" in r.extra and "rule=middle_area" in r.extra, f)
    any_rec(records, "防御姿态对应底盘功率倍率 0.5x",
            lambda r: r.node == "SendChassisCommand" and 37.0 <= r.bt_time <= 53.5 and
            "power_limit=10" in r.extra and "posture_scale=0.500000" in r.extra, f)

    any_rec(records, "TrackEnemy 窗口云台进入 TE",
            lambda r: r.node == "GetGimbalDecisions" and 55.0 <= r.bt_time <= 71.5 and r.gimbal == "TE", f)
    any_rec(records, "TrackEnemy 窗口 posture=1(rule=track_enemy)",
            lambda r: r.node == "GetSentryCmd" and 55.0 <= r.bt_time <= 71.5 and
            "mode=1" in r.extra and "rule=track_enemy" in r.extra, f)
    any_rec(records, "进攻姿态对应底盘功率倍率 0.5x",
            lambda r: r.node == "SendChassisCommand" and 55.0 <= r.bt_time <= 71.5 and
            "power_limit=10" in r.extra and "posture_scale=0.500000" in r.extra, f)

    any_rec(records, "丢失目标但仍在 middle_area 时 posture 回到 2(rule=middle_area)",
            lambda r: r.node == "GetSentryCmd" and 73.0 <= r.bt_time <= 89.5 and
            "mode=2" in r.extra and "rule=middle_area" in r.extra, f)
    any_rec(records, "丢失目标后回防御姿态功率仍为 0.5x",
            lambda r: r.node == "SendChassisCommand" and 73.0 <= r.bt_time <= 89.5 and
            "power_limit=10" in r.extra and "posture_scale=0.500000" in r.extra, f)

    any_rec(records, "离开 middle_area 且继续 GMA 时 posture 回到 3(rule=goto_middle_area)",
            lambda r: r.node == "GetSentryCmd" and 91.0 <= r.bt_time <= 107.5 and
            "mode=3" in r.extra and "rule=goto_middle_area" in r.extra, f)
    any_rec(records, "离开 middle_area 后回移动姿态功率恢复 1.5x",
            lambda r: r.node == "SendChassisCommand" and 91.0 <= r.bt_time <= 107.5 and
            "power_limit=30" in r.extra and "posture_scale=1.500000" in r.extra, f)
    return f


def check_capacitor_posture_transitions(records: List[Rec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "开局在 middle_area 外时底盘进入 GMA",
            lambda r: r.node == "GetChassisDecisions" and 1.0 <= r.bt_time <= 35.5 and r.chassis == "GMA", f)
    any_rec(records, "开局在 middle_area 外时 posture=3(rule=goto_middle_area)",
            lambda r: r.node == "GetSentryCmd" and 16.0 <= r.bt_time <= 35.5 and
            "mode=3" in r.extra and "rule=goto_middle_area" in r.extra, f)
    any_rec(records, "进入 middle_area 后 posture=2(rule=middle_area)",
            lambda r: r.node == "GetSentryCmd" and 37.0 <= r.bt_time <= 53.5 and
            "mode=2" in r.extra and "rule=middle_area" in r.extra, f)
    any_rec(records, "TrackEnemy 窗口云台进入 TE",
            lambda r: r.node == "GetGimbalDecisions" and 55.0 <= r.bt_time <= 71.5 and r.gimbal == "TE", f)
    any_rec(records, "TrackEnemy 窗口 posture=1(rule=track_enemy)",
            lambda r: r.node == "GetSentryCmd" and 55.0 <= r.bt_time <= 71.5 and
            "mode=1" in r.extra and "rule=track_enemy" in r.extra, f)
    any_rec(records, "丢失目标但仍在 middle_area 时 posture 回到 2(rule=middle_area)",
            lambda r: r.node == "GetSentryCmd" and 73.0 <= r.bt_time <= 89.5 and
            "mode=2" in r.extra and "rule=middle_area" in r.extra, f)
    any_rec(records, "离开 middle_area 且继续 GMA 时 posture 回到 3(rule=goto_middle_area)",
            lambda r: r.node == "GetSentryCmd" and 91.0 <= r.bt_time <= 107.5 and
            "mode=3" in r.extra and "rule=goto_middle_area" in r.extra, f)
    return f


def check_attack_phase(records: List[Rec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "攻击阶段存在 AMR 底盘模式",
            lambda r: r.node == "GetChassisDecisions" and 0.0 <= r.bt_time <= 29.5 and r.chassis == "AMR", f)
    any_rec(records, "攻击阶段存在 TrackEnemy",
            lambda r: r.node == "GetGimbalDecisions" and 0.0 <= r.bt_time <= 29.5 and r.gimbal == "TE", f)
    any_rec(records, "攻击阶段存在 Push",
            lambda r: r.node == "GetShooterDecisions" and 0.0 <= r.bt_time <= 29.5 and r.booster == "P", f)
    any_rec(records, "丢失目标窗口出现 FanSearchEnemy",
            lambda r: r.node == "GetGimbalDecisions" and 13.0 <= r.bt_time <= 18.0 and r.gimbal == "FSE", f)
    no_rec(records, "丢失目标窗口不应出现 Push",
           lambda r: r.node == "GetShooterDecisions" and 13.0 <= r.bt_time <= 18.0 and r.booster == "P", f)
    return f


def check_no_bullets(records: List[Rec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "无弹阶段存在 Ready 样本",
            lambda r: r.node == "GetShooterDecisions" and 1.0 <= r.bt_time <= 8.0 and r.booster == "R", f)
    no_rec(records, "无弹阶段不应 Push",
           lambda r: r.node == "GetShooterDecisions" and 1.0 <= r.bt_time <= 8.0 and r.booster == "P", f)
    any_rec(records, "补弹后出现 Push",
            lambda r: r.node == "GetShooterDecisions" and 9.0 <= r.bt_time <= 14.0 and r.booster == "P", f)
    any_rec(records, "再次无弹后回到 Ready",
            lambda r: r.node == "GetShooterDecisions" and 15.0 <= r.bt_time <= 24.0 and r.booster == "R", f)
    no_rec(records, "再次无弹后不应 Push",
           lambda r: r.node == "GetShooterDecisions" and 15.0 <= r.bt_time <= 24.0 and r.booster == "P", f)
    return f


def check_hp_urgent(records: List[Rec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "血量骤降后进入 GHRA",
            lambda r: r.node == "GetChassisDecisions" and 5.0 <= r.bt_time <= 20.0 and r.chassis == "GHRA", f)
    any_rec(records, "补给阶段进入 own_supply_area",
            lambda r: 15.0 <= r.bt_time <= 25.0 and r.area == "own_supply_area", f)
    return f


def check_conflict_chaos(records: List[Rec]) -> List[str]:
    f: List[str] = []
    no_rec(records, "敌方在补给区无敌阶段不应 Push",
           lambda r: r.node == "GetShooterDecisions" and 5.0 <= r.bt_time <= 8.0 and r.booster == "P", f)
    any_rec(records, "目标离开无敌区后恢复 Push",
            lambda r: r.node == "GetShooterDecisions" and 9.0 <= r.bt_time <= 11.0 and r.booster == "P", f)
    any_rec(records, "低血冲突阶段出现 GHRA",
            lambda r: r.node == "GetChassisDecisions" and 11.0 <= r.bt_time <= 14.0 and r.chassis == "GHRA", f)
    any_rec(records, "断联阶段底盘进入 ABH",
            lambda r: r.node == "GetChassisDecisions" and 14.0 <= r.bt_time <= 18.0 and r.chassis == "ABH", f)
    any_rec(records, "断联阶段射手进入 Stop",
            lambda r: r.node == "GetShooterDecisions" and 14.0 <= r.bt_time <= 18.0 and r.booster == "S", f)
    no_rec(records, "无弹阶段不应 Push",
           lambda r: r.node == "GetShooterDecisions" and 18.0 <= r.bt_time <= 21.0 and r.booster == "P", f)
    any_rec(records, "补弹恢复后再次 Push",
            lambda r: r.node == "GetShooterDecisions" and 22.0 <= r.bt_time <= 24.0 and r.booster == "P", f)
    any_rec(records, "死亡阶段 ReviveIfDead=dead",
            lambda r: r.node == "ReviveIfDead" and r.status == "FAILURE" and 24.0 <= r.bt_time <= 24.6 and "revive=dead" in r.extra, f)
    any_rec(records, "复活弱态期间保持 Ready",
            lambda r: r.node == "GetShooterDecisions" and 29.0 <= r.bt_time <= 35.0 and r.booster == "R", f)
    no_rec(records, "复活弱态期间不应 Push",
           lambda r: r.node == "GetShooterDecisions" and 29.0 <= r.bt_time <= 35.0 and r.booster == "P", f)
    any_rec(records, "稳定补给+HP增长后恢复 Push",
            lambda r: r.node == "GetShooterDecisions" and 36.0 <= r.bt_time <= 42.0 and r.booster == "P", f)
    return f


def check_control_area_dirty_gate(records: List[Rec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "控制区有效基线存在 Chase",
            lambda r: r.node == "GetChassisDecisions" and 1.0 <= r.bt_time <= 8.5 and r.chassis == "C", f)
    any_rec(records, "短时停发 + 保持窗口内仍沿用 Chase",
            lambda r: r.node == "GetChassisDecisions" and 9.5 <= r.bt_time <= 11.5 and r.chassis == "C", f)
    no_rec(records, "超窗失效后不应继续 Chase/UnChase",
           lambda r: r.node == "GetChassisDecisions" and 15.5 <= r.bt_time <= 17.8 and r.chassis in {"C", "UC"}, f)
    no_rec(records, "越界 robot_position 不应继续 Chase/UnChase",
           lambda r: r.node == "GetChassisDecisions" and 18.5 <= r.bt_time <= 21.8 and r.chassis in {"C", "UC"}, f)
    no_rec(records, "NaN/Inf robot_position 不应继续 Chase/UnChase",
           lambda r: r.node == "GetChassisDecisions" and 22.5 <= r.bt_time <= 25.8 and r.chassis in {"C", "UC"}, f)
    any_rec(records, "恢复有效 robot_position 后恢复 Chase",
            lambda r: r.node == "GetChassisDecisions" and 26.5 <= r.bt_time <= 31.5 and r.chassis == "C", f)
    return f


def check_enemy_revive_invincible_dirty_on(records: List[Rec]) -> List[str]:
    f: List[str] = []
    no_rec(records, "开启复活无敌检测时 freshly-resurrected 窗口不应 TrackEnemy",
           lambda r: r.node == "GetGimbalDecisions" and 3.0 <= r.bt_time <= 9.0 and r.gimbal == "TE", f)
    no_rec(records, "开启复活无敌检测时 freshly-resurrected 窗口不应 Push",
           lambda r: r.node == "GetShooterDecisions" and 3.0 <= r.bt_time <= 9.0 and r.booster == "P", f)
    any_rec(records, "freshly-resurrected 窗口后恢复 TrackEnemy",
            lambda r: r.node == "GetGimbalDecisions" and 10.5 <= r.bt_time <= 12.5 and r.gimbal == "TE", f)
    any_rec(records, "freshly-resurrected 窗口后恢复 Push",
            lambda r: r.node == "GetShooterDecisions" and 10.5 <= r.bt_time <= 12.5 and r.booster == "P", f)
    no_rec(records, "NaN/Inf track 窗口不应 TrackEnemy",
           lambda r: r.node == "GetGimbalDecisions" and 13.5 <= r.bt_time <= 15.5 and r.gimbal == "TE", f)
    no_rec(records, "NaN/Inf track 窗口不应 Push",
           lambda r: r.node == "GetShooterDecisions" and 13.5 <= r.bt_time <= 15.5 and r.booster == "P", f)
    any_rec(records, "恢复有效 track 后再次 TrackEnemy",
            lambda r: r.node == "GetGimbalDecisions" and 16.5 <= r.bt_time <= 19.0 and r.gimbal == "TE", f)
    any_rec(records, "恢复有效 track 后再次 Push",
            lambda r: r.node == "GetShooterDecisions" and 16.5 <= r.bt_time <= 19.0 and r.booster == "P", f)
    return f


def check_enemy_revive_invincible_dirty_off(records: List[Rec]) -> List[str]:
    f: List[str] = []
    any_rec(records, "关闭复活无敌检测时 freshly-resurrected 窗口应允许 TrackEnemy",
            lambda r: r.node == "GetGimbalDecisions" and 3.0 <= r.bt_time <= 9.0 and r.gimbal == "TE", f)
    any_rec(records, "关闭复活无敌检测时 freshly-resurrected 窗口应允许 Push",
            lambda r: r.node == "GetShooterDecisions" and 3.0 <= r.bt_time <= 9.0 and r.booster == "P", f)
    no_rec(records, "NaN/Inf track 窗口不应 TrackEnemy",
           lambda r: r.node == "GetGimbalDecisions" and 13.5 <= r.bt_time <= 15.5 and r.gimbal == "TE", f)
    no_rec(records, "NaN/Inf track 窗口不应 Push",
           lambda r: r.node == "GetShooterDecisions" and 13.5 <= r.bt_time <= 15.5 and r.booster == "P", f)
    any_rec(records, "恢复有效 track 后再次 TrackEnemy",
            lambda r: r.node == "GetGimbalDecisions" and 16.5 <= r.bt_time <= 19.0 and r.gimbal == "TE", f)
    any_rec(records, "恢复有效 track 后再次 Push",
            lambda r: r.node == "GetShooterDecisions" and 16.5 <= r.bt_time <= 19.0 and r.booster == "P", f)
    return f


def check_rulebook_full_chain(records: List[Rec]) -> List[str]:
    f: List[str] = []
    chassis_records = [r for r in records if r.node == "GetChassisDecisions"]
    if not chassis_records:
        return ["缺少证据: 未检测到 GetChassisDecisions 日志"]

    seen = {r.chassis for r in chassis_records}
    expected = {"GMA", "PMR", "C", "UC", "AMR", "GHRA", "ABH", "CSG", "AS"}
    missing = sorted(expected - seen)
    if missing:
        f.append("未覆盖到底盘模式: " + ", ".join(missing))

    min_t = min(r.bt_time for r in chassis_records)
    max_t = max(r.bt_time for r in chassis_records)
    if min_t > 5.0:
        f.append(f"比赛早期样本不足: 最早底盘日志时间={min_t:.2f}s")
    if max_t < 299.0:
        f.append(f"比赛末段样本不足: 最晚底盘日志时间={max_t:.2f}s")
    return f


def check_rulebook_full_chain_dirty(records: List[Rec]) -> List[str]:
    f = check_rulebook_full_chain(records)
    any_rec(records, "dirty 全链路基线存在 Chase",
            lambda r: r.node == "GetChassisDecisions" and 71.0 <= r.bt_time <= 83.0 and r.chassis == "C", f)
    any_rec(records, "dirty#1 短时停发仍沿用 Chase",
            lambda r: r.node == "GetChassisDecisions" and 86.5 <= r.bt_time <= 91.0 and r.chassis == "C", f)
    no_rec(records, "dirty#1 控制区超窗后不应继续 Chase/UnChase",
           lambda r: r.node == "GetChassisDecisions" and 94.0 <= r.bt_time <= 115.5 and r.chassis in {"C", "UC"}, f)
    no_rec(records, "dirty#1 freshly-resurrected 窗口不应 TrackEnemy",
           lambda r: r.node == "GetGimbalDecisions" and 129.0 <= r.bt_time <= 135.5 and r.gimbal == "TE", f)
    no_rec(records, "dirty#1 freshly-resurrected 窗口不应 Push",
           lambda r: r.node == "GetShooterDecisions" and 129.0 <= r.bt_time <= 135.5 and r.booster == "P", f)
    any_rec(records, "dirty#1 复活无敌窗口后恢复 Push",
            lambda r: r.node == "GetShooterDecisions" and 137.0 <= r.bt_time <= 146.5 and r.booster == "P", f)
    any_rec(records, "dirty#1 低血优先进入 GHRA",
            lambda r: r.node == "GetChassisDecisions" and 148.0 <= r.bt_time <= 156.0 and r.chassis == "GHRA", f)
    any_rec(records, "dirty#1 裁判断联进入 ABH",
            lambda r: r.node == "GetChassisDecisions" and 156.0 <= r.bt_time <= 166.0 and r.chassis == "ABH", f)
    any_rec(records, "dirty#1 裁判断联射手进入 Stop",
            lambda r: r.node == "GetShooterDecisions" and 156.0 <= r.bt_time <= 166.0 and r.booster == "S", f)
    no_rec(records, "dirty#2 控制区超窗后不应继续 Chase/UnChase",
           lambda r: r.node == "GetChassisDecisions" and 188.0 <= r.bt_time <= 193.5 and r.chassis in {"C", "UC"}, f)
    any_rec(records, "dirty#2 恢复有效数据后进入 UC",
            lambda r: r.node == "GetChassisDecisions" and 194.0 <= r.bt_time <= 201.5 and r.chassis == "UC", f)
    no_rec(records, "dirty#2 freshly-resurrected 窗口不应 TrackEnemy",
           lambda r: r.node == "GetGimbalDecisions" and 203.0 <= r.bt_time <= 209.5 and r.gimbal == "TE", f)
    no_rec(records, "dirty#2 freshly-resurrected 窗口不应 Push",
           lambda r: r.node == "GetShooterDecisions" and 203.0 <= r.bt_time <= 209.5 and r.booster == "P", f)
    any_rec(records, "dirty#2 复活无敌窗口后恢复 Push",
            lambda r: r.node == "GetShooterDecisions" and 210.0 <= r.bt_time <= 219.5 and r.booster == "P", f)
    any_rec(records, "dirty#2 低血优先进入 GHRA",
            lambda r: r.node == "GetChassisDecisions" and 220.0 <= r.bt_time <= 228.0 and r.chassis == "GHRA", f)
    any_rec(records, "dirty#2 裁判断联进入 ABH",
            lambda r: r.node == "GetChassisDecisions" and 228.0 <= r.bt_time <= 238.0 and r.chassis == "ABH", f)
    any_rec(records, "dirty#2 裁判断联射手进入 Stop",
            lambda r: r.node == "GetShooterDecisions" and 228.0 <= r.bt_time <= 238.0 and r.booster == "S", f)
    any_rec(records, "赛后在线窗口进入 CSG",
            lambda r: r.node == "GetChassisDecisions" and 300.0 <= r.bt_time <= 305.5 and r.chassis == "CSG", f)
    any_rec(records, "赛后裁判断联窗口进入 AS",
            lambda r: r.node == "GetChassisDecisions" and 300.0 <= r.bt_time <= 312.0 and r.chassis == "AS", f)
    return f


CHECKERS = {
    "full_match": check_full_match,
    "rulebook_full_chain": check_rulebook_full_chain,
    "rulebook_full_chain_dirty": check_rulebook_full_chain_dirty,
    "chase_gate_020b": check_chase_gate_020b,
    "control_area_dirty_gate": check_control_area_dirty_gate,
    "posture_transitions": check_posture_transitions,
    "capacitor_middle_area_demo": check_capacitor_posture_transitions,
    "capacitor_middle_area_disabled": check_capacitor_posture_transitions,
    "referee_offline": check_referee_offline,
    "attack_phase": check_attack_phase,
    "no_bullets": check_no_bullets,
    "hp_urgent": check_hp_urgent,
    "enemy_invincible_transition": check_enemy_invincible_transition,
    "enemy_revive_invincible_dirty_on": check_enemy_revive_invincible_dirty_on,
    "enemy_revive_invincible_dirty_off": check_enemy_revive_invincible_dirty_off,
    "weak_rfid_jitter": check_weak_rfid_jitter,
    "weak_localization_bias_supply": check_weak_localization_bias_supply,
    "conflict_chaos": check_conflict_chaos,
    "death_revive": check_weak_rfid_jitter,
}


def main() -> int:
    if len(sys.argv) != 3:
        print("用法: python3 verify_bt_behavior.py <scenario> <bt_log_file>")
        return 2
    scenario = sys.argv[1]
    log_path = sys.argv[2]
    if scenario not in CHECKERS:
        print(f"[SKIP] 未定义校验器: {scenario}")
        return 0
    records = parse_log(log_path)
    cap_records = parse_cap_log(log_path)
    if not records:
        print(f"[FAIL] {scenario}: 未解析到任何 BT 日志记录")
        return 1

    failures = CHECKERS[scenario](records)
    if scenario == "capacitor_middle_area_demo":
        failures.extend(check_capacitor_middle_area_demo(records, cap_records))
    elif scenario == "capacitor_middle_area_disabled":
        failures.extend(check_capacitor_middle_area_disabled(records, cap_records))
    if failures:
        print(f"[FAIL] {scenario}: {len(failures)} 项")
        for item in failures:
            print(f"  - {item}")
        return 1

    print(f"[PASS] {scenario}: 所有断言通过 ({len(records)} 条 BT 记录)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
