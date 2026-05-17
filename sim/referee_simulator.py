#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Referee-only match simulator for rm_behavior_tree.

Only publishes referee-system topics under /rm_referee/*.
No TF/odom/dbus/track are published here.

Default timeline follows RMUL 2026 3V3 flow:
- 180s PREPARATION
- 15s SELF_CHECKING
- 5s COUNTDOWN (represented as SELF_CHECKING in game_progress)
- 300s IN_BATTLE
- CALCULATION after battle ends
"""

import argparse
from typing import Callable, List

import rospy

from rm_msgs.msg import (
    GameStatus,
    GameRobotStatus,
    GameRobotHp,
    EventData,
    BulletAllowance,
    RfidStatus,
    SentryInfo,
    Buff,
    RadarToSentry,
    PowerHeatData,
)


class MatchEvent:
    def __init__(self, t: float, desc: str, fn: Callable):
        self.t = t
        self.desc = desc
        self.fn = fn
        self.triggered = False


class RefereeState:
    PRE_COMPETITION = 1
    PREPARATION = 2
    SELF_CHECKING = 3
    IN_BATTLE = 4
    CALCULATION = 5

    def __init__(self):
        # RMUL 3V3 timeline.
        self.preparation_duration_sec = 180.0
        self.self_check_duration_sec = 15.0
        self.countdown_duration_sec = 5.0
        self.match_duration_sec = 300.0
        self.battle_start_sec = (
            self.preparation_duration_sec
            + self.self_check_duration_sec
            + self.countdown_duration_sec
        )
        self.total_timeline_sec = self.battle_start_sec + self.match_duration_sec

        self.game_progress = self.PRE_COMPETITION
        self.stage_remain_time = self.preparation_duration_sec
        self.present_time = 0.0

        self.robot_id = 7
        self.remain_hp = 400
        self.max_hp = 400
        self.robot_hp = {
            "red_1": 150,
            "red_2": 200,
            "red_3": 300,
            "red_4": 300,
            "red_5": 300,
            "red_7": 400,
            "red_outpost": 1500,
            "red_base": 5000,
        }

        self.central_point_state = 0
        self.bullet_17mm = 300
        self.in_supply_zone = False
        self.referee_online = True

        self.radar_robot_id = 0
        self.radar_pos_x = 0.0
        self.radar_pos_y = 0.0

    @property
    def in_battle(self) -> bool:
        return self.game_progress == self.IN_BATTLE

    def update_stage_and_time(self, elapsed: float) -> None:
        if elapsed < self.preparation_duration_sec:
            self.game_progress = self.PREPARATION
            self.stage_remain_time = self.preparation_duration_sec - elapsed
            self.present_time = 0.0
            return

        if elapsed < self.preparation_duration_sec + self.self_check_duration_sec:
            self.game_progress = self.SELF_CHECKING
            self.stage_remain_time = (
                self.preparation_duration_sec + self.self_check_duration_sec - elapsed
            )
            self.present_time = 0.0
            return

        if elapsed < self.battle_start_sec:
            # GameStatus has no explicit countdown enum in current rm_msgs.
            self.game_progress = self.SELF_CHECKING
            self.stage_remain_time = self.battle_start_sec - elapsed
            self.present_time = 0.0
            return

        if elapsed < self.total_timeline_sec:
            self.game_progress = self.IN_BATTLE
            battle_elapsed = elapsed - self.battle_start_sec
            self.present_time = max(0.0, min(self.match_duration_sec, battle_elapsed))
            self.stage_remain_time = self.match_duration_sec - self.present_time
            return

        self.game_progress = self.CALCULATION
        self.stage_remain_time = 0.0
        self.present_time = self.match_duration_sec


def build_events() -> List[MatchEvent]:
    s = []

    def mk(t, desc, fn):
        s.append(MatchEvent(t, desc, fn))

    def start_battle(st: RefereeState):
        st.game_progress = RefereeState.IN_BATTLE
        st.central_point_state = 0
        st.bullet_17mm = 300
        st.remain_hp = st.max_hp
        st.robot_hp["red_7"] = st.max_hp

    def our_control(st: RefereeState):
        st.central_point_state = 1

    def enemy_control(st: RefereeState):
        st.central_point_state = 2

    def hp_urgent(st: RefereeState):
        st.remain_hp = 40
        st.robot_hp["red_7"] = 40
        st.in_supply_zone = False

    def enter_supply(st: RefereeState):
        st.in_supply_zone = True

    def recover(st: RefereeState):
        st.remain_hp = st.max_hp
        st.robot_hp["red_7"] = st.max_hp
        st.in_supply_zone = False
        st.central_point_state = 0

    def ammo_low(st: RefereeState):
        st.bullet_17mm = 30

    def ammo_recover(st: RefereeState):
        st.bullet_17mm = 200

    def offline_in_battle(st: RefereeState):
        st.referee_online = False

    def online_recover(st: RefereeState):
        st.referee_online = True
        st.central_point_state = 0

    def end_battle(st: RefereeState):
        st.game_progress = RefereeState.CALCULATION
        st.referee_online = True

    def offline_after_battle(st: RefereeState):
        st.referee_online = False

    def finish(st: RefereeState):
        st.referee_online = True

    # RMUL full flow (absolute timeline seconds):
    # 0-180 preparation, 180-195 self-check, 195-200 countdown, 200-500 battle.
    battle_t0 = 200.0

    mk(0.0, "preparation_start", lambda st: setattr(st, "game_progress", RefereeState.PREPARATION))
    mk(180.0, "self_check_start", lambda st: setattr(st, "game_progress", RefereeState.SELF_CHECKING))
    mk(195.0, "countdown_start", lambda st: setattr(st, "game_progress", RefereeState.SELF_CHECKING))

    mk(battle_t0 + 0, "battle_start", start_battle)
    mk(battle_t0 + 45, "our_control_center", our_control)
    mk(battle_t0 + 90, "enemy_control_center", enemy_control)
    mk(battle_t0 + 120, "hp_urgent", hp_urgent)
    mk(battle_t0 + 135, "enter_supply", enter_supply)
    mk(battle_t0 + 150, "recover", recover)
    mk(battle_t0 + 175, "ammo_low", ammo_low)
    mk(battle_t0 + 190, "ammo_recover", ammo_recover)
    mk(battle_t0 + 220, "referee_offline_in_battle", offline_in_battle)
    mk(battle_t0 + 235, "referee_online_recover", online_recover)
    mk(battle_t0 + 300, "battle_end", end_battle)
    mk(battle_t0 + 306, "referee_offline_after_battle", offline_after_battle)
    mk(battle_t0 + 312, "finish", finish)
    return s


class RefereeSimulator:
    def __init__(self, speed: float):
        rospy.init_node("referee_simulator", anonymous=False)
        self.speed = speed
        self.state = RefereeState()
        self.events = build_events()
        self.events.sort(key=lambda e: e.t)

        self.pub_game = rospy.Publisher("/rm_referee/game_status", GameStatus, queue_size=1)
        self.pub_robot = rospy.Publisher("/rm_referee/game_robot_status", GameRobotStatus, queue_size=1)
        self.pub_hp = rospy.Publisher("/rm_referee/game_robot_hp", GameRobotHp, queue_size=1)
        self.pub_event = rospy.Publisher("/rm_referee/event_data", EventData, queue_size=1)
        self.pub_bullet = rospy.Publisher("/rm_referee/bullet_allowance_data", BulletAllowance, queue_size=1)
        self.pub_rfid = rospy.Publisher("/rm_referee/rfid_status_data", RfidStatus, queue_size=1)
        self.pub_sentry = rospy.Publisher("/rm_referee/sentry_info", SentryInfo, queue_size=1)
        self.pub_buff = rospy.Publisher("/rm_referee/robot_buff", Buff, queue_size=1)
        self.pub_radar = rospy.Publisher("/rm_referee/radar_to_sentry", RadarToSentry, queue_size=1)
        self.pub_power = rospy.Publisher("/rm_referee/power_heat_data", PowerHeatData, queue_size=1)

        rospy.loginfo("[RefereeSim] speed=%.2fx events=%d", self.speed, len(self.events))

    def _build_game_status(self) -> GameStatus:
        msg = GameStatus()
        msg.game_progress = self.state.game_progress
        msg.stage_remain_time = max(0, int(self.state.stage_remain_time))
        msg.stamp = rospy.Time.now()
        return msg

    def _build_robot_status(self) -> GameRobotStatus:
        msg = GameRobotStatus()
        msg.robot_id = self.state.robot_id
        msg.remain_hp = self.state.remain_hp
        msg.max_hp = self.state.max_hp
        return msg

    def _build_game_robot_hp(self) -> GameRobotHp:
        msg = GameRobotHp()
        msg.ally_1_robot_hp = self.state.robot_hp["red_1"]
        msg.ally_2_robot_hp = self.state.robot_hp["red_2"]
        msg.ally_3_robot_hp = self.state.robot_hp["red_3"]
        msg.ally_4_robot_hp = self.state.robot_hp["red_4"]
        msg.ally_7_robot_hp = self.state.robot_hp["red_7"]
        msg.ally_outpost_hp = self.state.robot_hp["red_outpost"]
        msg.ally_base_hp = self.state.robot_hp["red_base"]
        return msg

    def _build_event_data(self) -> EventData:
        msg = EventData()
        msg.central_point_state = self.state.central_point_state
        return msg

    def _build_bullet_allowance(self) -> BulletAllowance:
        msg = BulletAllowance()
        msg.bullet_allowance_num_17_mm = self.state.bullet_17mm
        return msg

    def _build_rfid(self) -> RfidStatus:
        msg = RfidStatus()
        msg.non_overlapping_supplier_zone_state = self.state.in_supply_zone
        return msg

    def _build_radar(self) -> RadarToSentry:
        msg = RadarToSentry()
        msg.robot_ID = self.state.radar_robot_id
        msg.position_x = self.state.radar_pos_x
        msg.position_y = self.state.radar_pos_y
        return msg

    def _publish_referee(self):
        if self.state.referee_online:
            self.pub_game.publish(self._build_game_status())
            self.pub_robot.publish(self._build_robot_status())
            self.pub_hp.publish(self._build_game_robot_hp())
            self.pub_event.publish(self._build_event_data())
            self.pub_bullet.publish(self._build_bullet_allowance())
            self.pub_rfid.publish(self._build_rfid())
            self.pub_sentry.publish(SentryInfo())
            self.pub_buff.publish(Buff())
            self.pub_radar.publish(self._build_radar())

            pwr = PowerHeatData()
            pwr.stamp = rospy.Time.now()
            pwr.chassis_power_buffer = 60
            self.pub_power.publish(pwr)
        else:
            # Keep publishing stale power_heat to actively mark referee offline in Subscriber.
            pwr = PowerHeatData()
            pwr.stamp = rospy.Time.now() - rospy.Duration(2.0)
            pwr.chassis_power_buffer = 0
            self.pub_power.publish(pwr)

    def run(self):
        rate = rospy.Rate(50)
        for _ in range(100):
            if rospy.is_shutdown():
                return
            self._publish_referee()
            rate.sleep()

        sim_start = rospy.Time.now().to_sec()
        rospy.loginfo("[RefereeSim] running...")

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            elapsed = (now - sim_start) * self.speed

            self.state.update_stage_and_time(elapsed)

            for ev in self.events:
                if not ev.triggered and elapsed >= ev.t:
                    ev.triggered = True
                    rospy.loginfo("[RefereeSim] %.1fs event=%s", elapsed, ev.desc)
                    ev.fn(self.state)

            self._publish_referee()

            if elapsed > self.state.total_timeline_sec + 30.0:
                break
            rate.sleep()

        rospy.loginfo("[RefereeSim] done.")


def main():
    parser = argparse.ArgumentParser(description="Referee-only simulator")
    parser.add_argument("--speed", type=float, default=3.0, help="simulation speed factor")
    args, _ = parser.parse_known_args()

    sim = RefereeSimulator(speed=args.speed)
    sim.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
