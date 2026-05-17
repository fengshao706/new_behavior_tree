#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仅发送裁判系统信息的展示脚本。

目标：
  只发布 /rm_referee/* 相关话题，给外部 TF、/track 或其它演示节点提供裁判基线。

注意：
  这个脚本不再发送 TF、/track、/odom、/rm_ecat_hw/dbus。
  因此它本身不能单独驱动完整的姿态链路，只负责“裁判系统侧输入”。

用法：
  source /home/ROS_NOETIC_WS/rm_ws/devel/setup.bash
  python3 /home/ROS_NOETIC_WS/rm_ws/src/rm_sentry/decision/rm_behavior_tree/sim/posture_demo_driver.py

可选：
  python3 posture_demo_driver.py --print-plan
"""

import argparse
from dataclasses import dataclass
from typing import List, Optional

ROS_IMPORT_ERROR = None
try:
    import rospy
    from std_msgs.msg import String

    from rm_msgs.msg import (
        BulletAllowance,
        EventData,
        GameRobotHp,
        GameRobotStatus,
        GameStatus,
        PowerHeatData,
        RfidStatus,
        SentryCmd,
    )
except ModuleNotFoundError as exc:
    ROS_IMPORT_ERROR = exc


@dataclass(frozen=True)
class Phase:
    start_sec: float
    name: str
    central_point_state: int
    remain_hp: int
    bullet_17mm: int
    game_progress: int


PHASES: List[Phase] = [
    Phase(
        start_sec=0.0,
        name="阶段1: 对局中, 中心点未占领",
        central_point_state=0,
        remain_hp=400,
        bullet_17mm=240,
        game_progress=4,
    ),
    Phase(
        start_sec=8.0,
        name="阶段2: 对局中, 中心点我方占领",
        central_point_state=1,
        remain_hp=400,
        bullet_17mm=240,
        game_progress=4,
    ),
    Phase(
        start_sec=16.0,
        name="阶段3: 对局中, 中心点敌方占领",
        central_point_state=2,
        remain_hp=400,
        bullet_17mm=240,
        game_progress=4,
    ),
    Phase(
        start_sec=24.0,
        name="阶段4: 对局中, 中心点双方占领",
        central_point_state=3,
        remain_hp=400,
        bullet_17mm=240,
        game_progress=4,
    ),
    Phase(
        start_sec=32.0,
        name="阶段5: 收尾, 回到中心点未占领",
        central_point_state=0,
        remain_hp=400,
        bullet_17mm=240,
        game_progress=4,
    ),
]


class RefereeOnlyDemoDriver:
    def __init__(self, phase_hold_sec: float):
        rospy.init_node("posture_demo_driver", anonymous=False)
        self.phase_hold_sec = phase_hold_sec
        self.total_demo_sec = PHASES[-1].start_sec + self.phase_hold_sec + 3.0

        self.robot_id = 7
        self.remain_hp = PHASES[0].remain_hp
        self.max_hp = 400
        self.bullet_17mm = PHASES[0].bullet_17mm
        self.central_point_state = PHASES[0].central_point_state
        self.game_progress = PHASES[0].game_progress
        self.last_phase_name: Optional[str] = None
        self.last_posture_cmd: Optional[int] = None

        self.pub_game = rospy.Publisher("/rm_referee/game_status", GameStatus, queue_size=1)
        self.pub_robot = rospy.Publisher("/rm_referee/game_robot_status", GameRobotStatus, queue_size=1)
        self.pub_robot_hp = rospy.Publisher("/rm_referee/game_robot_hp", GameRobotHp, queue_size=1)
        self.pub_event = rospy.Publisher("/rm_referee/event_data", EventData, queue_size=1)
        self.pub_bullet = rospy.Publisher("/rm_referee/bullet_allowance_data", BulletAllowance, queue_size=1)
        self.pub_rfid = rospy.Publisher("/rm_referee/rfid_status_data", RfidStatus, queue_size=1)
        self.pub_power = rospy.Publisher("/rm_referee/power_heat_data", PowerHeatData, queue_size=1)

        rospy.Subscriber("/sentry_cmd", SentryCmd, self._on_sentry_cmd)
        rospy.Subscriber("/behavior_tree/log", String, self._on_bt_log)

    def _on_sentry_cmd(self, msg):
        if msg.posture_cmd == self.last_posture_cmd:
            return
        self.last_posture_cmd = msg.posture_cmd
        names = {0: "无", 1: "攻击", 2: "防守", 3: "移动"}
        rospy.loginfo("[RefereeDemo] /sentry_cmd posture=%s(%d)", names.get(msg.posture_cmd, "?"), msg.posture_cmd)

    def _on_bt_log(self, msg):
        text = msg.data
        if "GetSentryCmd" in text:
            rospy.loginfo("[RefereeDemo] %s", text)

    def _publish_referee(self, elapsed_sec: float):
        game = GameStatus()
        game.game_progress = self.game_progress
        game.stage_remain_time = max(0, int(300 - elapsed_sec))
        self.pub_game.publish(game)

        robot = GameRobotStatus()
        robot.robot_id = self.robot_id
        robot.remain_hp = self.remain_hp
        robot.max_hp = self.max_hp
        self.pub_robot.publish(robot)

        robot_hp = GameRobotHp()
        robot_hp.ally_7_robot_hp = self.remain_hp
        self.pub_robot_hp.publish(robot_hp)

        event = EventData()
        event.central_point_state = self.central_point_state
        self.pub_event.publish(event)

        bullet = BulletAllowance()
        bullet.bullet_allowance_num_17_mm = self.bullet_17mm
        self.pub_bullet.publish(bullet)

        rfid = RfidStatus()
        rfid.non_overlapping_supplier_zone_state = False
        self.pub_rfid.publish(rfid)

        power = PowerHeatData()
        power.stamp = rospy.Time.now()
        power.chassis_power_buffer = 60
        self.pub_power.publish(power)

    def _apply_phase(self, phase: Phase):
        self.central_point_state = phase.central_point_state
        self.remain_hp = phase.remain_hp
        self.bullet_17mm = phase.bullet_17mm
        self.game_progress = phase.game_progress
        if phase.name != self.last_phase_name:
            self.last_phase_name = phase.name
            rospy.loginfo(
                "[RefereeDemo] %s | game_progress=%d | central_point_state=%d | hp=%d | bullets=%d",
                phase.name,
                phase.game_progress,
                phase.central_point_state,
                phase.remain_hp,
                phase.bullet_17mm,
            )

    def _find_phase(self, elapsed_sec: float) -> Phase:
        phase = PHASES[0]
        for candidate in PHASES:
            if elapsed_sec >= candidate.start_sec:
                phase = candidate
            else:
                break
        return phase

    def run(self):
        rospy.sleep(1.0)
        rospy.loginfo("[RefereeDemo] 开始仅裁判系统演示，总时长 %.1fs", self.total_demo_sec)
        rate = rospy.Rate(30)
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            elapsed_sec = rospy.Time.now().to_sec() - start_time
            if elapsed_sec > self.total_demo_sec:
                break

            phase = self._find_phase(elapsed_sec)
            self._apply_phase(phase)
            self._publish_referee(elapsed_sec)
            rate.sleep()

        rospy.loginfo("[RefereeDemo] 演示结束")


def print_plan(phase_hold_sec: float):
    print("裁判系统发送时间表:")
    for idx, phase in enumerate(PHASES, start=1):
        end_sec = phase.start_sec + phase_hold_sec
        print(
            f"{idx}. {phase.start_sec:>5.1f}s - {end_sec:>5.1f}s | {phase.name} | "
            f"game_progress={phase.game_progress} | central_point_state={phase.central_point_state} | "
            f"hp={phase.remain_hp} | bullets={phase.bullet_17mm}"
        )


def main():
    parser = argparse.ArgumentParser(description="仅发送裁判系统信息的演示脚本")
    parser.add_argument("--phase-hold-sec", type=float, default=8.0, help="每个阶段保持时长，默认 8s")
    parser.add_argument("--print-plan", action="store_true", help="只打印阶段计划，不启动 ROS 发布")
    args = parser.parse_args()

    if args.phase_hold_sec < 3.0:
        args.phase_hold_sec = 3.0

    rebuilt = []
    for idx, phase in enumerate(PHASES):
        rebuilt.append(
            Phase(
                start_sec=idx * args.phase_hold_sec,
                name=phase.name,
                central_point_state=phase.central_point_state,
                remain_hp=phase.remain_hp,
                bullet_17mm=phase.bullet_17mm,
                game_progress=phase.game_progress,
            )
        )
    PHASES[:] = rebuilt

    if args.print_plan:
        print_plan(args.phase_hold_sec)
        return

    if ROS_IMPORT_ERROR is not None:
        raise SystemExit(
            "ROS Python environment not ready. Please run:\n"
            "source /home/ROS_NOETIC_WS/rm_ws/devel/setup.bash"
        )

    driver = RefereeOnlyDemoDriver(args.phase_hold_sec)
    driver.run()


if __name__ == "__main__":
    main()
