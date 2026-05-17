#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bt_monitor.py
行为树状态实时监控工具

功能：
  1. 订阅 /behavior_tree/log，解析行为树节点状态
  2. 订阅 /sentry_cmd，显示哨兵姿态指令
  3. 订阅 /custom_info，显示攻击目标信息
  4. 定时打印模式状态表格
  5. 将仿真结束后的状态变化日志保存到文件

用法：
  rosrun rm_behavior_tree bt_monitor.py [--output <log_file.txt>]
"""

import re
import sys
import time
import argparse
from collections import defaultdict, deque
from datetime import datetime
from typing import Optional, Dict, List, Tuple

import rospy
from std_msgs.msg import String
from rm_msgs.msg import SentryCmd


# ============================================================
# ANSI 颜色
# ============================================================
RESET   = "\033[0m"
BOLD    = "\033[1m"
RED     = "\033[91m"
GREEN   = "\033[92m"
YELLOW  = "\033[93m"
CYAN    = "\033[96m"
MAGENTA = "\033[95m"
BLUE    = "\033[94m"
WHITE   = "\033[97m"
DIM     = "\033[2m"

CHASSIS_COLOR = {
    'GMA':  GREEN,    # GotoMiddleArea
    'AMR':  RED,      # AttackMidRobot
    'PMR':  BLUE,     # ProtectMidRobot
    'CSG':  DIM,      # ChassisSlowGyro
    'AS':   YELLOW,   # AbnormalStill
    'ABH':  YELLOW,   # AbnormalBackHome
    'GHRA': CYAN,     # GotoHpReturnArea
    'C':    MAGENTA,  # Chase
    'UC':   DIM,      # UnChase
    'AD':   RED,      # AvoidDrone
}
GIMBAL_COLOR = {
    'YSR': DIM,
    'AS':  YELLOW,
    'LTF': CYAN,
    'RSE': GREEN,
    'FSE': MAGENTA,
    'IG':  BLUE,
    'TE':  RED,
}
BOOSTER_COLOR = {
    'S': DIM,
    'R': YELLOW,
    'P': RED,
}

# 全称映射
CHASSIS_FULL = {
    'GMA': 'GotoMiddleArea', 'AMR': 'AttackMidRobot', 'PMR': 'ProtectMidRobot',
    'CSG': 'ChassisSlowGyro', 'AS': 'AbnormalStill', 'ABH': 'AbnormalBackHome',
    'GHRA': 'GotoHpReturnArea', 'C': 'Chase', 'UC': 'UnChase', 'AD': 'AvoidDrone',
}
GIMBAL_FULL = {
    'YSR': 'YawSlowRound', 'AS': 'AbnormalStill', 'LTF': 'LidarTowardsFront',
    'RSE': 'RoundSearchEnemy', 'FSE': 'FanSearchEnemy', 'IG': 'InverseGimbal', 'TE': 'TrackEnemy',
}
BOOSTER_FULL = {
    'S': 'Stop', 'R': 'Ready', 'P': 'Push',
}

# 日志行正则
LOG_RE = re.compile(
    r'\[BT\] (?P<node>\S+)\s+(?P<status>SUCCESS|FAILURE|RUNNING)'
    r'\s+C:(?P<chassis>\S+)\s+G:(?P<gimbal>\S+)\s+B:(?P<booster>\S+)'
    r'\s+pos=\((?P<px>[\d.\-]+),(?P<py>[\d.\-]+)\)@(?P<area>\S+)'
    r'\s+Enemy:(?P<enemy>\S+)\s+Time:(?P<time>[\d.]+)'
    r'(?:\s+(?P<extra>.*))?'
)


def colored(c: str, txt: str) -> str:
    return f"{c}{txt}{RESET}"


class BTMonitor:

    def __init__(self, output_file: Optional[str] = None):
        rospy.init_node('bt_monitor', anonymous=False)

        self.output_file = output_file
        self.log_lines: List[str] = []

        # 最近状态
        self.last_chassis  = '?'
        self.last_gimbal   = '?'
        self.last_booster  = '?'
        self.last_area     = '?'
        self.last_enemy    = '?'
        self.last_time     = 0.0
        self.last_pos      = (0.0, 0.0)

        # 模式切换历史
        self.chassis_history: deque  = deque(maxlen=200)
        self.gimbal_history : deque  = deque(maxlen=200)
        self.booster_history: deque  = deque(maxlen=200)

        # 模式持续时间统计
        self.chassis_duration: Dict[str, float] = defaultdict(float)
        self.gimbal_duration : Dict[str, float] = defaultdict(float)
        self.booster_duration: Dict[str, float] = defaultdict(float)
        self._chassis_enter_time: Optional[float] = None
        self._gimbal_enter_time : Optional[float] = None
        self._booster_enter_time: Optional[float] = None

        # 节点状态统计
        self.node_call_count: Dict[str, int] = defaultdict(int)
        self.node_fail_count: Dict[str, int] = defaultdict(int)

        rospy.Subscriber('/behavior_tree/log', String, self._on_log)
        rospy.Subscriber('/sentry_cmd', SentryCmd, self._on_sentry_cmd)
        rospy.Subscriber('/custom_info', String, self._on_custom)

        self._print_header()
        # 定时打印监控摘要
        rospy.Timer(rospy.Duration(10.0), self._print_summary)

    def _print_header(self):
        print(f"\n{BOLD}{'='*72}{RESET}")
        print(f"{BOLD}  rm_behavior_tree 行为状态实时监控  |  {datetime.now().strftime('%H:%M:%S')}{RESET}")
        print(f"{BOLD}{'='*72}{RESET}")
        print(f"{BOLD}  底盘缩写: GMA=GotoMiddleArea AMR=AttackMidRobot PMR=ProtectMidRobot{RESET}")
        print(f"{BOLD}           GHRA=GotoHpReturnArea C=Chase UC=UnChase CSG=SlowGyro{RESET}")
        print(f"{BOLD}  云台缩写: TE=TrackEnemy RSE=RoundSearch FSE=FanSearch IG=Inverse{RESET}")
        print(f"{BOLD}  射击缩写: S=Stop R=Ready P=Push{RESET}")
        print(f"{BOLD}{'─'*72}{RESET}\n")

    def _on_log(self, msg: String):
        text = msg.data
        m = LOG_RE.search(text)
        if not m:
            return

        node    = m.group('node')
        status  = m.group('status')
        chassis = m.group('chassis')
        gimbal  = m.group('gimbal')
        booster = m.group('booster')
        area    = m.group('area')
        enemy   = m.group('enemy')
        bt_time = float(m.group('time'))
        px      = float(m.group('px'))
        py      = float(m.group('py'))
        extra   = m.group('extra') or ''

        self.node_call_count[node] += 1
        if status == 'FAILURE':
            self.node_fail_count[node] += 1

        self.last_area  = area
        self.last_enemy = enemy
        self.last_time  = bt_time
        self.last_pos   = (px, py)

        changed = False

        # 底盘变化检测
        if chassis != self.last_chassis and node == 'GetChassisDecisions':
            self._update_duration_stats('chassis', chassis)
            ts = f"[{bt_time:6.1f}s]"
            old_c = colored(CHASSIS_COLOR.get(self.last_chassis, ''), CHASSIS_FULL.get(self.last_chassis, self.last_chassis))
            new_c = colored(CHASSIS_COLOR.get(chassis, ''), CHASSIS_FULL.get(chassis, chassis))
            line = f"{ts} {BOLD}底盘切换:{RESET} {old_c} → {new_c}  @{area}"
            print(line)
            self._append_log(f"[{bt_time:.1f}s] CHASSIS: {self.last_chassis}→{chassis} @{area}")
            self.chassis_history.append((bt_time, self.last_chassis, chassis, area))
            self.last_chassis = chassis
            changed = True

        # 云台变化检测
        if gimbal != self.last_gimbal and node == 'GetGimbalDecisions':
            self._update_duration_stats('gimbal', gimbal)
            ts = f"[{bt_time:6.1f}s]"
            old_g = colored(GIMBAL_COLOR.get(self.last_gimbal, ''), GIMBAL_FULL.get(self.last_gimbal, self.last_gimbal))
            new_g = colored(GIMBAL_COLOR.get(gimbal, ''), GIMBAL_FULL.get(gimbal, gimbal))
            line = f"{ts} {BOLD}云台切换:{RESET} {old_g} → {new_g}  @{area}"
            print(line)
            self._append_log(f"[{bt_time:.1f}s] GIMBAL: {self.last_gimbal}→{gimbal} @{area}")
            self.gimbal_history.append((bt_time, self.last_gimbal, gimbal, area))
            self.last_gimbal = gimbal
            changed = True

        # 射击变化检测
        if booster != self.last_booster and node == 'GetShooterDecisions':
            self._update_duration_stats('booster', booster)
            ts = f"[{bt_time:6.1f}s]"
            old_b = colored(BOOSTER_COLOR.get(self.last_booster, ''), BOOSTER_FULL.get(self.last_booster, self.last_booster))
            new_b = colored(BOOSTER_COLOR.get(booster, ''), BOOSTER_FULL.get(booster, booster))
            line = f"{ts} {BOLD}射击切换:{RESET} {old_b} → {new_b}"
            print(line)
            self._append_log(f"[{bt_time:.1f}s] BOOSTER: {self.last_booster}→{booster}")
            self.booster_history.append((bt_time, self.last_booster, booster, area))
            self.last_booster = booster
            changed = True

        # FAILURE 节点特别提示
        if status == 'FAILURE':
            ts = f"[{bt_time:6.1f}s]"
            line = f"{ts} {RED}{BOLD}[FAIL] {node}{RESET}{RED} {extra}{RESET}"
            print(line)
            self._append_log(f"[{bt_time:.1f}s] FAIL: {node} {extra}")

    def _on_sentry_cmd(self, msg: SentryCmd):
        posture_map = {0: ('无', DIM), 1: ('攻击🗡', RED), 2: ('防守🛡', BLUE), 3: ('移动🏃', GREEN)}
        label, color = posture_map.get(msg.posture_cmd, ('?', ''))
        revive = f" {RED}[请求复活]{RESET}" if msg.confirm_respawn else ''
        print(f"         {BOLD}哨兵指令:{RESET} 姿态={colored(color, label)}{revive}")
        self._append_log(f"SENTRY_CMD: posture={msg.posture_cmd} revive={msg.confirm_respawn}")

    def _on_custom(self, msg: String):
        print(f"         {CYAN}{BOLD}攻击目标:{RESET} {msg.data}")

    def _update_duration_stats(self, mode_type: str, new_mode: str):
        now = self.last_time
        if mode_type == 'chassis':
            if self._chassis_enter_time is not None:
                dur = now - self._chassis_enter_time
                self.chassis_duration[self.last_chassis] += dur
            self._chassis_enter_time = now
        elif mode_type == 'gimbal':
            if self._gimbal_enter_time is not None:
                dur = now - self._gimbal_enter_time
                self.gimbal_duration[self.last_gimbal] += dur
            self._gimbal_enter_time = now
        elif mode_type == 'booster':
            if self._booster_enter_time is not None:
                dur = now - self._booster_enter_time
                self.booster_duration[self.last_booster] += dur
            self._booster_enter_time = now

    def _append_log(self, line: str):
        self.log_lines.append(line)
        if self.output_file:
            with open(self.output_file, 'a') as f:
                f.write(line + '\n')

    def _print_summary(self, event=None):
        t = self.last_time
        print(f"\n{BOLD}{'─'*60}{RESET}")
        print(f"{BOLD}  [摘要 @{t:.1f}s]  区域:{self.last_area}  敌方:{self.last_enemy}  "
              f"位置:({self.last_pos[0]:.2f},{self.last_pos[1]:.2f}){RESET}")
        # 当前状态
        c = colored(CHASSIS_COLOR.get(self.last_chassis, ''), CHASSIS_FULL.get(self.last_chassis, self.last_chassis))
        g = colored(GIMBAL_COLOR.get(self.last_gimbal, ''), GIMBAL_FULL.get(self.last_gimbal, self.last_gimbal))
        b = colored(BOOSTER_COLOR.get(self.last_booster, ''), BOOSTER_FULL.get(self.last_booster, self.last_booster))
        print(f"  当前状态: 底盘={c}  云台={g}  射击={b}")

        # 节点调用统计（只打印有FAILURE的）
        failures = {k: v for k, v in self.node_fail_count.items() if v > 0}
        if failures:
            print(f"  {RED}FAILURE 节点: " + " | ".join(f"{k}:{v}次" for k, v in failures.items()) + RESET)

        # 底盘模式切换历史（最近5条）
        if self.chassis_history:
            print(f"  底盘切换历史（最近5条）:")
            for entry in list(self.chassis_history)[-5:]:
                bt_t, old, new, area = entry
                print(f"    [{bt_t:.1f}s] {CHASSIS_FULL.get(old, old)} → {CHASSIS_FULL.get(new, new)} @{area}")

        # 模式持续时间
        if any(self.chassis_duration.values()):
            print(f"  底盘模式持续: " + " ".join(
                f"{CHASSIS_FULL.get(k,k)}:{v:.1f}s" for k, v in sorted(self.chassis_duration.items(), key=lambda x: -x[1])
            ))
        print(f"{BOLD}{'─'*60}{RESET}\n")

    def run(self):
        rospy.loginfo("[BTMonitor] 行为树监控器已启动，订阅 /behavior_tree/log ...")
        rospy.spin()
        self._print_final_report()

    def _print_final_report(self):
        print(f"\n{BOLD}{'='*60}{RESET}")
        print(f"{BOLD}  仿真结束报告{RESET}")
        print(f"{BOLD}{'='*60}{RESET}")
        print(f"节点调用统计:")
        for node, cnt in sorted(self.node_call_count.items(), key=lambda x: -x[1]):
            fail = self.node_fail_count.get(node, 0)
            fail_str = colored(RED, f" (FAIL:{fail})") if fail else ""
            print(f"  {node}: {cnt}次{fail_str}")
        print(f"\n底盘模式分布:")
        for mode, dur in sorted(self.chassis_duration.items(), key=lambda x: -x[1]):
            print(f"  {CHASSIS_FULL.get(mode, mode):25s}: {dur:6.1f}s ({100*dur/max(self.last_time,1):.1f}%)")
        print(f"\n云台模式分布:")
        for mode, dur in sorted(self.gimbal_duration.items(), key=lambda x: -x[1]):
            print(f"  {GIMBAL_FULL.get(mode, mode):25s}: {dur:6.1f}s ({100*dur/max(self.last_time,1):.1f}%)")
        print(f"\n射击模式分布:")
        for mode, dur in sorted(self.booster_duration.items(), key=lambda x: -x[1]):
            print(f"  {BOOSTER_FULL.get(mode, mode):10s}: {dur:6.1f}s ({100*dur/max(self.last_time,1):.1f}%)")
        if self.output_file:
            print(f"\n日志已保存到: {self.output_file}")
        print(f"{BOLD}{'='*60}{RESET}\n")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default=None, help='输出日志文件路径')
    args, _ = parser.parse_known_args()
    try:
        BTMonitor(output_file=args.output).run()
    except rospy.ROSInterruptException:
        pass
