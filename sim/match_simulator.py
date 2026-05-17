#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RMUC 2026 比赛仿真驱动脚本
模拟完整5分钟比赛的裁判系统 / 遥控器 / 目标跟踪数据，
驱动 rm_behavior_tree 节点并通过监控话题验证决策输出。

用法：
  rosrun rm_behavior_tree match_simulator.py [--scenario <name>] [--speed <factor>]

场景 (--scenario):
  full_match      (默认) 完整5分钟比赛流程
  rulebook_full_chain 按 2026 规则手册节奏覆盖完整底盘模式链路
  rulebook_full_chain_dirty 在规则全链路中插入 dirty 输入与优先级竞争
  chase_gate_020b 通过 0x020B 等效 robot_position 验证追击门槛
  control_area_dirty_gate 验证控制区门槛的超时/无数据/异常值
  posture_transitions posture 姿态切换场景
  capacitor_middle_area_demo 中路超电开启场景
  capacitor_middle_area_disabled 中路超电总开关关闭对照场景
  hp_urgent       血量紧急 → 补血场景
  death_revive    死亡 → 复活场景
  referee_offline 裁判系统断联场景
  attack_phase    持续攻击中心点场景
  no_bullets      弹药耗尽场景
  enemy_revive_invincible_dirty_on/off 复活无敌检测开关 dirty 对照
  conflict_chaos  多条件冲突与混乱切换压力场景
"""

import sys
import time
import math
import argparse
import threading
import os
from typing import Optional

import rospy
import tf2_ros
import geometry_msgs.msg

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped

# rm_msgs
from rm_msgs.msg import (
    DbusData, GameStatus, GameRobotStatus, GameRobotHp,
    EventData, BulletAllowance, RfidStatus, TrackData,
    SentryInfo, Buff, RadarToSentry, SentryCmd,
    PowerHeatData, PowerManagementSampleAndStatusData,
    RobotsPositionData, ManualToReferee
)

# ============================================================
# 颜色输出工具
# ============================================================
RESET  = "\033[0m"
BOLD   = "\033[1m"
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
MAGENTA= "\033[95m"
BLUE   = "\033[94m"


def cprint(color: str, msg: str) -> None:
    print(f"{color}{msg}{RESET}", flush=True)


# ============================================================
# 比赛场景事件定义
# ============================================================
class MatchEvent:
    """在 present_time 触发一次场景事件"""
    def __init__(self, present_time: float, description: str, apply_fn):
        self.present_time = present_time   # 距比赛开始的秒数
        self.description  = description
        self.apply_fn     = apply_fn       # callable(state: GameState)
        self.triggered    = False


class GameState:
    """仿真中的完整比赛状态，可被事件修改"""

    # game_progress 常量（参考 rm_msgs/GameStatus）
    PRE_COMPETITION = 1
    PREPARATION     = 2
    SELF_CHECKING   = 3
    IN_BATTLE       = 4
    CALCULATION     = 5

    def __init__(self):
        self.game_progress    : int   = self.PRE_COMPETITION
        self.stage_remain_time: float = 300.0   # 剩余时间(s)
        self.present_time     : float = 0.0      # 已流逝时间(s)

        # 机器人状态 (假设红方哨兵, robot_id=7)
        self.robot_id    : int   = 7
        self.remain_hp   : int   = 400
        self.max_hp      : int   = 400

        # 全场血量 (简化，只关注己方)
        self.robot_hp    : dict  = {
            'red_1': 150, 'red_2': 200, 'red_3': 300,
            'red_4': 300, 'red_5': 300, 'red_7': 400,
            'blue_1':150, 'blue_2':200,'blue_3':300,
            'blue_4':300,'blue_5':300,'blue_7':400,
        }

        # 事件数据
        self.central_point_state: int = 0   # 0未占 1我方 2敌方 3双方

        # 弹药
        self.bullet_17mm: int = 500

        # Buff
        self.buff_value: int = 0

        # RFID
        self.in_supply_zone: bool = False

        # 雷达目标
        self.radar_robot_id : int   = 0
        self.radar_pos_x    : float = 0.0
        self.radar_pos_y    : float = 0.0

        # 跟踪目标
        self.track_id       : int   = 0
        self.track_pos_x    : float = 5.0
        self.track_pos_y    : float = 2.0
        self.track_vx       : float = 0.0
        self.track_vy       : float = 0.0

        # 机器人自身位置 (map 坐标系)
        self.robot_x  : float = 2.846
        self.robot_y  : float = 3.758
        self.robot_yaw: float = 0.0

        # 超电状态
        self.capacity_remain_charge: float = 80.0
        self.capacity_recent_charge_power: float = 20.0
        self.capacity_discharge_power: int = 40
        self.capacity_state_machine_running_state: int = 2
        self.power_management_protection_info: int = 0
        self.power_management_topology: int = 0

        # 0x020B /robot_position 对应的我方机器人位置
        self.hero_x       : float = 0.85
        self.hero_y       : float = 1.05
        self.engineer_x   : float = 0.85
        self.engineer_y   : float = 1.05
        self.standard_3_x : float = 0.85
        self.standard_3_y : float = 1.05
        self.standard_4_x : float = 0.85
        self.standard_4_y : float = 1.05

        # 遥控器状态 (s_r: UP=1=auto)
        self.rc_online  : bool  = True
        self.rc_s_r     : int   = DbusData.UP   # auto 模式

        # 裁判系统在线 (由 referee_is_online_ 内部维持，这里控制是否发布)
        self.referee_online: bool = True
        self.publish_robot_position: bool = True
        self.publish_track: bool = True

        # 哨兵指令
        self.sentry_confirm_respawn: int = 0
        self.sentry_posture_cmd    : int = 0
        self.manual_power_limit_state: Optional[int] = None

    @property
    def is_in_battle(self) -> bool:
        return self.game_progress == self.IN_BATTLE


def set_control_area_occupied(state: GameState, occupied: bool):
    if occupied:
        ally_x, ally_y = 2.846, 3.758
    else:
        ally_x, ally_y = 0.85, 1.05
    state.hero_x, state.hero_y = ally_x, ally_y
    state.engineer_x, state.engineer_y = 0.85, 1.05
    state.standard_3_x, state.standard_3_y = 0.85, 1.05
    state.standard_4_x, state.standard_4_y = 0.85, 1.05


def set_control_area_out_of_bounds(state: GameState):
    state.hero_x, state.hero_y = 999.0, 999.0
    state.engineer_x, state.engineer_y = -999.0, 999.0
    state.standard_3_x, state.standard_3_y = 999.0, -999.0
    state.standard_4_x, state.standard_4_y = -999.0, -999.0


def set_control_area_nan_inf(state: GameState):
    state.hero_x, state.hero_y = math.nan, math.nan
    state.engineer_x, state.engineer_y = math.inf, 0.0
    state.standard_3_x, state.standard_3_y = 0.0, math.inf
    state.standard_4_x, state.standard_4_y = -math.inf, math.nan


def set_track_target(state: GameState, robot_id: int, x: float, y: float, tracking: bool = True):
    state.track_id = robot_id
    state.track_pos_x = x
    state.track_pos_y = y
    state.track_vx = 0.0
    state.track_vy = 0.0
    if not tracking:
        state.track_id = 0


# ============================================================
# 场景定义
# ============================================================
def build_full_match_events(state: GameState):
    """构建完整比赛事件列表"""
    events = []

    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    # 0s: 比赛开始
    def start_battle(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        cprint(GREEN, f"\n{'='*60}")
        cprint(GREEN, f"  [比赛开始] IN_BATTLE — present_time=0s")
        cprint(GREEN, f"{'='*60}\n")
    events.append(mk(0, "比赛开始", start_battle))

    # 5s: 机器人进入中间区域
    def enter_middle(s: GameState):
        s.robot_x, s.robot_y = 2.846, 3.758
        cprint(CYAN, f"[{s.present_time:.0f}s] 机器人进入 middle_area (2.85, 3.76)")
    events.append(mk(5, "进入中间区域", enter_middle))

    # 15s: 中心点未被占领，开始巡逻
    def gyro_on(s: GameState):
        cprint(CYAN, f"[{s.present_time:.0f}s] present_time>10s → 小陀螺自动启动")
    events.append(mk(11, "小陀螺启动", gyro_on))

    # 25s: 我方占领中心点
    def our_capture(s: GameState):
        s.central_point_state = 1
        cprint(MAGENTA, f"[{s.present_time:.0f}s] 中心点被我方占领 (central_point_state=1) → ProtectMidRobot")
    events.append(mk(25, "我方占领中心点", our_capture))

    # 40s: 敌方出现跟踪目标
    def enemy_appears(s: GameState):
        s.track_id = 3    # 红方步兵3号
        s.track_pos_x = 4.0
        s.track_pos_y = 2.0
        cprint(RED, f"[{s.present_time:.0f}s] 发现敌方 ID={s.track_id} → TrackEnemy, Booster=Push")
    events.append(mk(40, "发现跟踪目标", enemy_appears))

    # 55s: 敌方占领中心点
    def enemy_capture(s: GameState):
        s.central_point_state = 2
        cprint(RED, f"[{s.present_time:.0f}s] 中心点被敌方占领 (central_point_state=2) → AttackMidRobot")
    events.append(mk(55, "敌方占领中心点", enemy_capture))

    # 70s: 跟踪丢失
    def track_lost(s: GameState):
        s.track_id = 0
        cprint(YELLOW, f"[{s.present_time:.0f}s] 跟踪丢失 → FanSearchEnemy(AttackMidRobot期间)")
    events.append(mk(70, "跟踪丢失", track_lost))

    # 80s: 机器人受伤，血量降至250
    def hp_drop_250(s: GameState):
        s.remain_hp = 250
        s.robot_hp['red_7'] = 250
        cprint(YELLOW, f"[{s.present_time:.0f}s] 血量降至 {s.remain_hp} / {s.max_hp} → GotoHpReturnArea (hp<300 且距离>10m)")
    events.append(mk(80, "血量告急", hp_drop_250))

    # 90s: 机器人到达补给区，RFID触发
    def reach_supply(s: GameState):
        s.robot_x, s.robot_y = 0.0, 0.0   # own_supply_area (pos_detection_polygon [1,2,3,4])
        s.in_supply_zone = True
        cprint(GREEN, f"[{s.present_time:.0f}s] 进入补给区，RFID触发，开始补血")
    events.append(mk(90, "到达补给区", reach_supply))

    # 100s: 血量回满
    def hp_full(s: GameState):
        s.remain_hp = 400
        s.robot_hp['red_7'] = 400
        s.in_supply_zone = False
        # 离开补给区后回到己方后场待机区附近，避免区域判定长期停留在 own_supply_area
        s.robot_x, s.robot_y = 0.85, 1.05
        cprint(GREEN, f"[{s.present_time:.0f}s] 血量回满 {s.remain_hp} → 离开补给区，回到后场待机区")
    events.append(mk(100, "血量回满", hp_full))

    # 110s: 双方占领中心点
    def both_capture(s: GameState):
        s.central_point_state = 3
        cprint(MAGENTA, f"[{s.present_time:.0f}s] 双方占领中心点 (state=3) → AttackMidRobot")
    events.append(mk(110, "双方占领中心点", both_capture))

    # 130s: 重新出现跟踪目标
    def track_resumes(s: GameState):
        s.track_id    = 1   # 红方英雄
        s.track_pos_x = 3.5
        s.track_pos_y = 3.0
        cprint(RED, f"[{s.present_time:.0f}s] 重新发现英雄 ID={s.track_id} → TrackEnemy")
    events.append(mk(130, "重见目标", track_resumes))

    # 150s: 保守限制期开始
    def conservative_start(s: GameState):
        cprint(YELLOW, f"[{s.present_time:.0f}s] present_time≥150s → 进入保守追击限制期，更多区域禁止追击")
    events.append(mk(150, "保守限制期", conservative_start))

    # 180s: 弹量耗尽测试
    def bullets_empty(s: GameState):
        s.bullet_17mm = 0
        cprint(RED, f"[{s.present_time:.0f}s] 弹量耗尽 → isBulletsRemain=False → 抑制Chase")
    events.append(mk(180, "弹药耗尽", bullets_empty))

    # 190s: 补弹
    def bullets_refill(s: GameState):
        s.bullet_17mm = 200
        cprint(GREEN, f"[{s.present_time:.0f}s] 补弹完成 → bullet_17mm={s.bullet_17mm}")
    events.append(mk(190, "补弹完成", bullets_refill))

    # 210s: 机器人死亡
    def robot_die(s: GameState):
        s.remain_hp = 0
        s.robot_hp['red_7'] = 0
        s.track_id = 0
        cprint(RED, f"\n[{s.present_time:.0f}s] {'='*40}")
        cprint(RED, f"[{s.present_time:.0f}s] 机器人死亡! HP=0 → ReviveIfDead=FAILURE，停止控制器")
        cprint(RED, f"[{s.present_time:.0f}s] {'='*40}\n")
    events.append(mk(210, "机器人死亡", robot_die))

    # 218s: 机器人复活
    def robot_revive(s: GameState):
        s.remain_hp = int(0.2 * s.max_hp)
        s.robot_hp['red_7'] = s.remain_hp
        cprint(GREEN, f"[{s.present_time:.0f}s] 机器人复活! HP={s.remain_hp}(20%) → 进入无敌+虚弱，等待0.8s校准")
    events.append(mk(218, "机器人复活", robot_revive))

    # 224s: 复活后进入己方补给区，解除无敌/虚弱
    def revive_enter_supply(s: GameState):
        s.robot_x, s.robot_y = 0.0, 0.0
        s.in_supply_zone = True
        cprint(GREEN, f"[{s.present_time:.0f}s] 复活后到达己方补给区，RFID触发 → 解除无敌/虚弱")
    events.append(mk(224, "复活后进入补给区", revive_enter_supply))

    # 228s: 离开补给区并补到更高血量（模拟回血过程）
    def revive_leave_supply(s: GameState):
        s.in_supply_zone = False
        s.robot_x, s.robot_y = 0.85, 1.05
        s.remain_hp = 220
        s.robot_hp['red_7'] = s.remain_hp
        cprint(CYAN, f"[{s.present_time:.0f}s] 离开补给区，HP={s.remain_hp}，恢复正常作战资格")
    events.append(mk(228, "复活后离开补给区", revive_leave_supply))

    # 230s: 裁判系统断联
    def referee_offline(s: GameState):
        s.referee_online = False
        cprint(RED, f"[{s.present_time:.0f}s] 裁判系统断联! → AbnormalBackHome(如在战斗)")
    events.append(mk(230, "裁判系统断联", referee_offline))

    # 240s: 裁判系统恢复
    def referee_online(s: GameState):
        s.referee_online = True
        cprint(GREEN, f"[{s.present_time:.0f}s] 裁判系统恢复 → 正常决策恢复")
    events.append(mk(240, "裁判系统恢复", referee_online))

    # 260s: 血量再次告急
    def hp_urgent2(s: GameState):
        s.remain_hp = 100
        s.robot_hp['red_7'] = 100
        cprint(YELLOW, f"[{s.present_time:.0f}s] 血量告急 {s.remain_hp} → GotoHpReturnArea")
    events.append(mk(260, "终局血量告急", hp_urgent2))

    # 280s: 比赛结束前
    def near_end(s: GameState):
        s.remain_hp = 400
        cprint(CYAN, f"[{s.present_time:.0f}s] 比赛即将结束 (remain 20s)")
    events.append(mk(280, "比赛即将结束", near_end))

    # 300s: 比赛结束
    def end_battle(s: GameState):
        s.game_progress = GameState.CALCULATION
        cprint(GREEN, f"\n{'='*60}")
        cprint(GREEN, f"  [比赛结束] CALCULATION — 总用时 300s")
        cprint(GREEN, f"{'='*60}\n")
    events.append(mk(300, "比赛结束", end_battle))

    return events


def build_rulebook_full_chain_events(state: GameState):
    """
    基于 2026 规则手册「五分钟比赛阶段」设计的全链路场景：
    从比赛开始持续发布裁判话题到比赛结束，并覆盖关键底盘模式：
      GMA / PMR / C / UC / AMR / GHRA / ABH / CSG / AS
    """
    events = []

    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def set_control_area_occupied(s: GameState, occupied: bool):
        if occupied:
            s.hero_x, s.hero_y = 2.846, 3.758
        else:
            s.hero_x, s.hero_y = 0.85, 1.05
        s.engineer_x, s.engineer_y = 0.85, 1.05
        s.standard_3_x, s.standard_3_y = 0.85, 1.05
        s.standard_4_x, s.standard_4_y = 0.85, 1.05

    def start_battle(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.referee_online = True
        s.central_point_state = 0
        s.remain_hp = 400
        s.robot_hp['red_7'] = 400
        s.bullet_17mm = 320
        s.in_supply_zone = False
        s.track_id = 0
        s.robot_x, s.robot_y = 0.85, 1.05   # own_leisure_area
        set_control_area_occupied(s, False)
        cprint(GREEN, f"\n{'='*64}")
        cprint(GREEN, f"  [规则场景开始] 五分钟比赛阶段 IN_BATTLE")
        cprint(GREEN, f"{'='*64}\n")
    events.append(mk(0, "比赛开始（5分钟阶段）", start_battle))

    def mode_gma(s: GameState):
        s.central_point_state = 0
        s.track_id = 0
        s.robot_x, s.robot_y = 0.85, 1.05
        cprint(CYAN, f"[{s.present_time:.0f}s] central=0 → 预期 GMA")
    events.append(mk(20, "触发 GotoMiddleArea", mode_gma))

    def mode_pmr(s: GameState):
        s.central_point_state = 1
        s.track_id = 0
        s.robot_x, s.robot_y = 0.85, 1.05
        set_control_area_occupied(s, False)
        cprint(CYAN, f"[{s.present_time:.0f}s] central=1 + 无跟踪 → 预期 PMR")
    events.append(mk(45, "触发 ProtectMidRobot", mode_pmr))

    def mode_chase(s: GameState):
        s.central_point_state = 1
        s.bullet_17mm = 280
        s.track_id = 3
        s.track_pos_x, s.track_pos_y = 2.6, 4.2
        s.track_vx, s.track_vy = 0.0, 0.0
        s.robot_x, s.robot_y = 0.85, 1.05  # 非限制区
        set_control_area_occupied(s, True)
        cprint(RED, f"[{s.present_time:.0f}s] central=1 + TrackEnemy + 0x020B显示友方在控制区 → 预期 Chase(C)")
    events.append(mk(70, "触发 Chase", mode_chase))

    def mode_amr(s: GameState):
        s.central_point_state = 2
        s.track_id = 0
        s.robot_x, s.robot_y = 0.85, 1.05
        set_control_area_occupied(s, False)
        cprint(RED, f"[{s.present_time:.0f}s] central=2（敌方占点）→ 预期 AMR")
    events.append(mk(120, "触发 AttackMidRobot", mode_amr))

    def mode_ghra(s: GameState):
        s.central_point_state = 2
        s.remain_hp = 40
        s.robot_hp['red_7'] = 40
        s.in_supply_zone = False
        set_control_area_occupied(s, False)
        cprint(YELLOW, f"[{s.present_time:.0f}s] 低血量 HP={s.remain_hp} → 预期 GHRA")
    events.append(mk(150, "触发 GotoHpReturnArea", mode_ghra))

    def reach_supply(s: GameState):
        s.robot_x, s.robot_y = 0.0, 0.0
        s.in_supply_zone = True
        cprint(GREEN, f"[{s.present_time:.0f}s] 到达补给区，持续发布 RFID")
    events.append(mk(165, "到达补给区", reach_supply))

    def recover_hp(s: GameState):
        s.remain_hp = 400
        s.robot_hp['red_7'] = 400
        s.in_supply_zone = False
        s.central_point_state = 0
        s.track_id = 0
        s.robot_x, s.robot_y = 0.85, 1.05
        set_control_area_occupied(s, False)
        cprint(GREEN, f"[{s.present_time:.0f}s] 补血完成，回到常规作战区域")
    events.append(mk(178, "补血完成", recover_hp))

    def mode_unchase(s: GameState):
        s.central_point_state = 1
        s.bullet_17mm = 260
        s.track_id = 3
        s.track_pos_x, s.track_pos_y = 2.7, 4.1
        # 该点在当前图中常被判定为 own_protect_mid_robot_area，
        # 且在 conservative chase_restricted_zones 中，用于稳定触发 UC。
        s.robot_x, s.robot_y = 2.846, 3.758
        set_control_area_occupied(s, True)
        cprint(YELLOW, f"[{s.present_time:.0f}s] 0x020B显示友方仍在控制区，自己进入受限区域 → 预期 UC")
    events.append(mk(200, "触发 UnChase", mode_unchase))

    def mode_abh(s: GameState):
        s.referee_online = False
        cprint(RED, f"[{s.present_time:.0f}s] 比赛中裁判掉线 → 预期 ABH")
    events.append(mk(220, "触发 AbnormalBackHome", mode_abh))

    def referee_resume(s: GameState):
        s.referee_online = True
        s.central_point_state = 0
        s.robot_x, s.robot_y = 0.85, 1.05
        set_control_area_occupied(s, False)
        cprint(GREEN, f"[{s.present_time:.0f}s] 裁判恢复在线，回到常规决策")
    events.append(mk(235, "裁判恢复", referee_resume))

    def end_battle(s: GameState):
        s.game_progress = GameState.CALCULATION
        s.referee_online = True
        cprint(GREEN, f"\n{'='*64}")
        cprint(GREEN, f"  [比赛结束] CALCULATION（规则 5 分钟阶段结束）")
        cprint(GREEN, f"{'='*64}\n")
    events.append(mk(300, "比赛结束", end_battle))

    def mode_as(s: GameState):
        s.referee_online = False
        cprint(RED, f"[{s.present_time:.0f}s] 非战斗阶段裁判掉线 → 预期 AS")
    events.append(mk(306, "触发 AbnormalStill", mode_as))

    def end_tail(s: GameState):
        s.referee_online = True
        cprint(CYAN, f"[{s.present_time:.0f}s] 场景收尾，准备退出")
    events.append(mk(312, "收尾", end_tail))

    return events


def build_chase_gate_020b_events(state: GameState):
    events = []

    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def set_control_area_occupied(s: GameState, occupied: bool):
        if occupied:
            s.hero_x, s.hero_y = 2.846, 3.758
        else:
            s.hero_x, s.hero_y = 0.85, 1.05
        s.engineer_x, s.engineer_y = 0.85, 1.05
        s.standard_3_x, s.standard_3_y = 0.85, 1.05
        s.standard_4_x, s.standard_4_y = 0.85, 1.05

    def start_battle(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.referee_online = True
        s.robot_x, s.robot_y = 0.85, 1.05  # own_leisure_area，确保“自己不在控制区”
        s.central_point_state = 1
        s.bullet_17mm = 220
        s.track_id = 3
        s.track_pos_x = 3.4
        s.track_pos_y = 2.6
        set_control_area_occupied(s, True)
        cprint(GREEN, f"[{s.present_time:.0f}s] 我方占点 + 0x020B显示友方在控制区 → 预期 Chase")

    def ally_leave_control(s: GameState):
        set_control_area_occupied(s, False)
        cprint(YELLOW, f"[{s.present_time:.0f}s] 我方仍占点，但0x020B显示控制区无人 → 预期 ProtectMidRobot")

    def ally_return_control(s: GameState):
        set_control_area_occupied(s, True)
        cprint(CYAN, f"[{s.present_time:.0f}s] 0x020B显示友方重新进入控制区 → 预期恢复 Chase")

    def enemy_take_control(s: GameState):
        s.central_point_state = 2
        set_control_area_occupied(s, True)
        cprint(RED, f"[{s.present_time:.0f}s] 敌方占点，即使0x020B仍显示友方在区内 → 预期 AttackMidRobot")

    def end_battle(s: GameState):
        s.game_progress = GameState.CALCULATION
        cprint(GREEN, f"[{s.present_time:.0f}s] 测试结束")

    events.append(mk(0.0, "开始 0x020B 追击门槛测试", start_battle))
    events.append(mk(8.0, "控制区无人", ally_leave_control))
    events.append(mk(16.0, "友方重新进入控制区", ally_return_control))
    events.append(mk(24.0, "敌方重新占点", enemy_take_control))
    events.append(mk(30.0, "结束", end_battle))
    return events


def build_hp_urgent_events(state: GameState):
    """简化场景：聚焦血量告急与补血"""
    events = []
    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def start(s): s.game_progress = GameState.IN_BATTLE
    events.append(mk(0, "开始", start))

    def drop_hp(s):
        s.remain_hp = 150
        cprint(RED, f"[{s.present_time:.0f}s] 血量骤降至 150 → 距离补给区约 5m → threshold≈175")
    events.append(mk(5, "血量骤降", drop_hp))

    def reach(s):
        s.robot_x, s.robot_y = 0.0, 0.0
        s.in_supply_zone = True
    events.append(mk(15, "到达补给区", reach))

    def full(s):
        s.remain_hp = 400
        s.in_supply_zone = False
    events.append(mk(25, "补血完毕", full))

    def stop(s): s.game_progress = GameState.CALCULATION
    events.append(mk(60, "结束", stop))
    return events


def build_enemy_invincible_transition_events(state: GameState):
    """敌方目标先在敌方补给区（应抑制射击），后离开补给区（应恢复射击）"""
    events = []
    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def start(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.robot_x, s.robot_y = 0.85, 1.05
        s.bullet_17mm = 200
        s.central_point_state = 2
        cprint(GREEN, f"[{s.present_time:.0f}s] 开始敌方无敌区抑制测试")
    events.append(mk(0, "开始", start))

    def enemy_in_supply(s: GameState):
        s.track_id = 1
        s.track_pos_x, s.track_pos_y = 5.2, 9.2  # enemy_supply_area 内
        cprint(YELLOW, f"[{s.present_time:.0f}s] 敌方目标进入 enemy_supply_area → 应抑制 Push")
    events.append(mk(2, "敌方在补给区", enemy_in_supply))

    def enemy_leave_supply(s: GameState):
        s.track_pos_x, s.track_pos_y = 2.5, 6.2  # enemy_leisure_area / 非补给区
        cprint(CYAN, f"[{s.present_time:.0f}s] 敌方目标离开补给区 → 应恢复 Push")
    events.append(mk(10, "敌方离开补给区", enemy_leave_supply))

    def stop(s: GameState):
        s.game_progress = GameState.CALCULATION
        cprint(GREEN, f"[{s.present_time:.0f}s] 测试结束")
    events.append(mk(18, "结束", stop))
    return events


def build_weak_rfid_jitter_events(state: GameState):
    """复活后弱态期间出现误触发 RFID 抖动，验证不会提前解除弱态（期望）"""
    events = []
    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def start(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.robot_x, s.robot_y = 0.85, 1.05  # own_leisure_area
        s.bullet_17mm = 200
        s.central_point_state = 2
        cprint(GREEN, f"[{s.present_time:.0f}s] 开始复活弱态 + RFID抖动测试")
    events.append(mk(0, "开始", start))

    def track_before_death(s: GameState):
        s.track_id = 1
        s.track_pos_x, s.track_pos_y = 2.8, 4.2
        cprint(CYAN, f"[{s.present_time:.0f}s] 目标可见，正常具备 Push 条件")
    events.append(mk(2, "目标可见", track_before_death))

    def die(s: GameState):
        s.remain_hp = 0
        s.robot_hp['red_7'] = 0
        s.track_id = 0
        cprint(RED, f"[{s.present_time:.0f}s] 死亡 → ReviveIfDead 应阻断")
    events.append(mk(5, "死亡", die))

    def revive(s: GameState):
        s.remain_hp = int(0.2 * s.max_hp)
        s.robot_hp['red_7'] = s.remain_hp
        s.track_id = 1
        s.track_pos_x, s.track_pos_y = 2.8, 4.2  # 非补给区目标
        s.in_supply_zone = False
        cprint(GREEN, f"[{s.present_time:.0f}s] 复活(20%HP) → 进入弱态，射手应保持 Ready")
    events.append(mk(13, "复活", revive))

    def rfid_glitch_on(s: GameState):
        # 故意不改位姿（仍在 own_leisure_area），仅模拟裁判 RFID 瞬时误触发
        s.in_supply_zone = True
        cprint(YELLOW, f"[{s.present_time:.1f}s] RFID 抖动 ON（位置不在补给区）")
    events.append(mk(17.0, "RFID抖动开启", rfid_glitch_on))

    def rfid_glitch_off(s: GameState):
        s.in_supply_zone = False
        cprint(YELLOW, f"[{s.present_time:.1f}s] RFID 抖动 OFF")
    events.append(mk(18.0, "RFID抖动关闭", rfid_glitch_off))

    def stop(s: GameState):
        s.game_progress = GameState.CALCULATION
        cprint(GREEN, f"[{s.present_time:.0f}s] 测试结束")
    events.append(mk(22, "结束", stop))
    return events


def build_weak_localization_bias_supply_events(state: GameState):
    """复活后定位仍在 own_leisure_area，但 RFID 稳定且 HP 增长，应允许解除弱态"""
    events = []
    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def start(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.robot_x, s.robot_y = 0.85, 1.05  # 故意保持在 own_leisure_area（模拟定位偏差）
        s.bullet_17mm = 200
        s.central_point_state = 2
        cprint(GREEN, f"[{s.present_time:.0f}s] 开始复活弱态 + 定位偏差补给测试")
    events.append(mk(0, "开始", start))

    def track_on(s: GameState):
        s.track_id = 1
        s.track_pos_x, s.track_pos_y = 2.8, 4.2
        cprint(CYAN, f"[{s.present_time:.0f}s] 目标可见，满足恢复后 Push 的条件")
    events.append(mk(2, "目标可见", track_on))

    def die(s: GameState):
        s.remain_hp = 0
        s.robot_hp['red_7'] = 0
        s.track_id = 0
        cprint(RED, f"[{s.present_time:.0f}s] 死亡")
    events.append(mk(5, "死亡", die))

    def revive(s: GameState):
        s.remain_hp = int(0.2 * s.max_hp)
        s.robot_hp['red_7'] = s.remain_hp
        s.track_id = 1
        s.track_pos_x, s.track_pos_y = 2.8, 4.2
        s.in_supply_zone = False
        cprint(GREEN, f"[{s.present_time:.0f}s] 复活(20%%HP) → 弱态开始")
    events.append(mk(13, "复活", revive))

    def rfid_on(s: GameState):
        s.in_supply_zone = True
        cprint(YELLOW, f"[{s.present_time:.1f}s] RFID 稳定 ON（定位仍偏在 own_leisure_area）")
    events.append(mk(17.0, "RFID稳定开启", rfid_on))

    def hp_gain(s: GameState):
        s.remain_hp = 140
        s.robot_hp['red_7'] = s.remain_hp
        cprint(CYAN, f"[{s.present_time:.1f}s] 补给生效，HP 增长到 {s.remain_hp}（应可解除弱态）")
    events.append(mk(18.5, "HP增长", hp_gain))

    def rfid_off(s: GameState):
        s.in_supply_zone = False
        cprint(YELLOW, f"[{s.present_time:.1f}s] RFID OFF")
    events.append(mk(21.0, "RFID关闭", rfid_off))

    def stop(s: GameState):
        s.game_progress = GameState.CALCULATION
        cprint(GREEN, f"[{s.present_time:.0f}s] 测试结束")
    events.append(mk(24, "结束", stop))
    return events


def build_referee_offline_events(state: GameState):
    """裁判系统反复掉线/恢复，验证离线优先级与恢复行为"""
    events = []

    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def start(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.referee_online = True
        s.central_point_state = 2
        s.bullet_17mm = 220
        s.track_id = 1
        s.track_pos_x, s.track_pos_y = 2.8, 4.2
        s.robot_x, s.robot_y = 0.85, 1.05
        cprint(GREEN, f"[{s.present_time:.0f}s] 开始裁判断联场景")
    events.append(mk(0, "开始", start))

    def offline_1(s: GameState):
        s.referee_online = False
        cprint(RED, f"[{s.present_time:.0f}s] 裁判系统断联#1 → 预期 ABH/AS/Stop")
    events.append(mk(3, "裁判断联#1", offline_1))

    def online_1(s: GameState):
        s.referee_online = True
        s.track_id = 1
        s.bullet_17mm = 220
        cprint(GREEN, f"[{s.present_time:.0f}s] 裁判恢复#1 → 预期恢复 TrackEnemy + Push")
    events.append(mk(10, "裁判恢复#1", online_1))

    def offline_2(s: GameState):
        s.referee_online = False
        cprint(RED, f"[{s.present_time:.0f}s] 裁判系统断联#2 → 再次进入离线保守模式")
    events.append(mk(16, "裁判断联#2", offline_2))

    def online_2(s: GameState):
        s.referee_online = True
        s.track_id = 3
        s.track_pos_x, s.track_pos_y = 2.6, 4.1
        s.bullet_17mm = 180
        cprint(GREEN, f"[{s.present_time:.0f}s] 裁判恢复#2 → 继续作战")
    events.append(mk(22, "裁判恢复#2", online_2))

    def stop(s: GameState):
        s.game_progress = GameState.CALCULATION
        cprint(GREEN, f"[{s.present_time:.0f}s] 测试结束")
    events.append(mk(30, "结束", stop))
    return events


def build_attack_phase_events(state: GameState):
    """持续攻击中心点，验证 AMR/TE/P 及丢失后的搜索切换"""
    events = []

    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def start(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.referee_online = True
        s.central_point_state = 2
        s.robot_x, s.robot_y = 0.85, 1.05
        s.bullet_17mm = 260
        s.track_id = 3
        s.track_pos_x, s.track_pos_y = 2.8, 4.2
        cprint(GREEN, f"[{s.present_time:.0f}s] 开始持续攻击场景（预期 AMR 主导）")
    events.append(mk(0, "开始", start))

    def keep_attack(s: GameState):
        s.central_point_state = 3
        cprint(RED, f"[{s.present_time:.0f}s] 双方占点，仍应保持攻击姿态")
    events.append(mk(6, "保持攻击", keep_attack))

    def lose_track(s: GameState):
        s.track_id = 0
        cprint(YELLOW, f"[{s.present_time:.0f}s] 暂时丢失目标 → 预期 FanSearch/Ready")
    events.append(mk(12, "丢失目标", lose_track))

    def recover_track(s: GameState):
        s.track_id = 1
        s.track_pos_x, s.track_pos_y = 2.7, 4.0
        cprint(CYAN, f"[{s.present_time:.0f}s] 恢复目标 → 预期 TrackEnemy + Push")
    events.append(mk(18, "恢复目标", recover_track))

    def stop(s: GameState):
        s.game_progress = GameState.CALCULATION
        cprint(GREEN, f"[{s.present_time:.0f}s] 测试结束")
    events.append(mk(30, "结束", stop))
    return events


def build_posture_transitions_events(state: GameState):
    """验证 posture 随 GMA/middle_area/TrackEnemy 切换"""
    events = []

    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def start(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.referee_online = True
        s.central_point_state = 0
        s.robot_x, s.robot_y = 0.85, 1.05
        s.bullet_17mm = 240
        s.track_id = 0
        cprint(GREEN, f"[{s.present_time:.0f}s] 开局在 middle_area 外，无目标 → 预期 GMA + posture=移动")
    events.append(mk(0, "开局 GMA", start))

    def enter_middle(s: GameState):
        s.robot_x, s.robot_y = 2.846, 3.758
        s.track_id = 0
        cprint(CYAN, f"[{s.present_time:.0f}s] 进入 middle_area，无目标 → 预期 posture=防御")
    events.append(mk(36, "进入 middle_area", enter_middle))

    def track_enemy(s: GameState):
        s.track_id = 1
        s.track_pos_x, s.track_pos_y = 2.7, 4.0
        cprint(RED, f"[{s.present_time:.0f}s] middle_area 内获取有效目标 → 预期 TrackEnemy + posture=攻击")
    events.append(mk(54, "进入 TrackEnemy", track_enemy))

    def lose_track(s: GameState):
        s.track_id = 0
        cprint(YELLOW, f"[{s.present_time:.0f}s] 仍在 middle_area 但丢失目标 → 预期 posture=防御")
    events.append(mk(72, "丢失目标", lose_track))

    def leave_middle(s: GameState):
        s.robot_x, s.robot_y = 0.85, 1.05
        s.track_id = 0
        cprint(CYAN, f"[{s.present_time:.0f}s] 离开 middle_area，继续 GMA → 预期 posture=移动")
    events.append(mk(90, "离开 middle_area", leave_middle))

    def stop(s: GameState):
        s.game_progress = GameState.CALCULATION
        cprint(GREEN, f"[{s.present_time:.0f}s] 测试结束")
    events.append(mk(108, "结束", stop))
    return events


def build_no_bullets_events(state: GameState):
    """持续有目标但弹药为0，验证不会进入 Push；补弹后恢复 Push"""
    events = []

    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def start(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.referee_online = True
        s.central_point_state = 2
        s.robot_x, s.robot_y = 0.85, 1.05
        s.track_id = 1
        s.track_pos_x, s.track_pos_y = 2.9, 4.0
        s.bullet_17mm = 0
        cprint(GREEN, f"[{s.present_time:.0f}s] 开始无弹场景（有目标但 bullets=0）")
    events.append(mk(0, "开始", start))

    def refill(s: GameState):
        s.bullet_17mm = 120
        cprint(CYAN, f"[{s.present_time:.0f}s] 补弹完成 bullets={s.bullet_17mm} → 预期恢复 Push")
    events.append(mk(8, "补弹", refill))

    def empty_again(s: GameState):
        s.bullet_17mm = 0
        cprint(YELLOW, f"[{s.present_time:.0f}s] 再次耗尽弹药 → 应回 Ready")
    events.append(mk(14, "再次耗尽", empty_again))

    def stop(s: GameState):
        s.game_progress = GameState.CALCULATION
        cprint(GREEN, f"[{s.present_time:.0f}s] 测试结束")
    events.append(mk(24, "结束", stop))
    return events


def build_conflict_chaos_events(state: GameState):
    """多冲突混合：无敌区目标/断联/低血/无弹/死亡复活/RFID 抖动 连续交叉"""
    events = []

    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def start(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.referee_online = True
        s.central_point_state = 1
        s.robot_x, s.robot_y = 0.85, 1.05
        s.track_id = 3
        s.track_pos_x, s.track_pos_y = 2.8, 4.2
        s.bullet_17mm = 220
        s.remain_hp = 400
        s.robot_hp['red_7'] = 400
        s.in_supply_zone = False
        cprint(GREEN, f"[{s.present_time:.0f}s] 开始冲突混乱场景")
    events.append(mk(0, "开始", start))

    def enemy_supply(s: GameState):
        s.track_id = 1
        s.track_pos_x, s.track_pos_y = 5.2, 9.2
        cprint(YELLOW, f"[{s.present_time:.0f}s] 目标进入 enemy_supply_area（无敌）→ 不应 Push")
    events.append(mk(4, "敌方补给区无敌", enemy_supply))

    def enemy_leave_supply(s: GameState):
        s.track_pos_x, s.track_pos_y = 2.6, 6.2
        cprint(CYAN, f"[{s.present_time:.0f}s] 目标离开补给区 → 应恢复 Push")
    events.append(mk(8, "离开无敌区", enemy_leave_supply))

    def hp_urgent(s: GameState):
        s.remain_hp = 55
        s.robot_hp['red_7'] = 55
        s.central_point_state = 2
        cprint(YELLOW, f"[{s.present_time:.0f}s] 血量骤降至 {s.remain_hp} → 预期 GHRA")
    events.append(mk(11, "血量告急", hp_urgent))

    def referee_offline(s: GameState):
        s.referee_online = False
        cprint(RED, f"[{s.present_time:.0f}s] 裁判断联 + 低血并发 → 离线优先")
    events.append(mk(14, "裁判断联", referee_offline))

    def referee_back_no_bullets(s: GameState):
        s.referee_online = True
        s.bullet_17mm = 0
        s.track_id = 3
        cprint(CYAN, f"[{s.present_time:.0f}s] 裁判恢复但无弹药 → 仍不应 Push")
    events.append(mk(18, "恢复但无弹", referee_back_no_bullets))

    def bullets_back(s: GameState):
        s.bullet_17mm = 160
        cprint(GREEN, f"[{s.present_time:.0f}s] 补弹恢复 → 可再次 Push")
    events.append(mk(21, "补弹恢复", bullets_back))

    def die(s: GameState):
        s.remain_hp = 0
        s.robot_hp['red_7'] = 0
        s.track_id = 0
        cprint(RED, f"[{s.present_time:.0f}s] 死亡 → ReviveIfDead FAILURE")
    events.append(mk(24, "死亡", die))

    def revive(s: GameState):
        s.remain_hp = int(0.2 * s.max_hp)
        s.robot_hp['red_7'] = s.remain_hp
        s.track_id = 1
        s.track_pos_x, s.track_pos_y = 2.8, 4.2
        s.in_supply_zone = False
        cprint(GREEN, f"[{s.present_time:.0f}s] 复活(20%HP) → 弱态，射手应 Ready")
    events.append(mk(28, "复活", revive))

    def rfid_glitch_on(s: GameState):
        s.in_supply_zone = True
        cprint(YELLOW, f"[{s.present_time:.0f}s] RFID 误触发 ON（位置仍不在补给区）")
    events.append(mk(31, "RFID误触发ON", rfid_glitch_on))

    def rfid_glitch_off(s: GameState):
        s.in_supply_zone = False
        cprint(YELLOW, f"[{s.present_time:.0f}s] RFID 误触发 OFF")
    events.append(mk(32, "RFID误触发OFF", rfid_glitch_off))

    def stable_supply_and_hp_gain(s: GameState):
        s.robot_x, s.robot_y = 0.0, 0.0
        s.in_supply_zone = True
        s.remain_hp = 160
        s.robot_hp['red_7'] = 160
        cprint(CYAN, f"[{s.present_time:.0f}s] 稳定补给区 + HP增长 → 应解除弱态并恢复进攻")
    events.append(mk(35, "稳定补给解除弱态", stable_supply_and_hp_gain))

    def leave_supply(s: GameState):
        s.in_supply_zone = False
        s.robot_x, s.robot_y = 0.85, 1.05
        s.central_point_state = 2
        s.track_id = 3
        s.track_pos_x, s.track_pos_y = 2.7, 4.1
        cprint(GREEN, f"[{s.present_time:.0f}s] 离开补给区，恢复常规战斗")
    events.append(mk(38, "离开补给区", leave_supply))

    def stop(s: GameState):
        s.game_progress = GameState.CALCULATION
        cprint(GREEN, f"[{s.present_time:.0f}s] 场景结束")
    events.append(mk(42, "结束", stop))
    return events


def build_control_area_dirty_gate_events(state: GameState):
    events = []

    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def start(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.referee_online = True
        s.central_point_state = 1
        s.robot_x, s.robot_y = 0.85, 1.05
        s.bullet_17mm = 220
        s.publish_robot_position = True
        set_control_area_occupied(s, True)
        set_track_target(s, 3, 2.6, 4.1)
        cprint(GREEN, f"[{s.present_time:.0f}s] 开始控制区 dirty 门槛测试，基线应为 Chase")
    events.append(mk(0, "开始", start))

    def short_timeout(s: GameState):
        s.publish_robot_position = False
        cprint(YELLOW, f"[{s.present_time:.0f}s] 停发 /robot_position（短时），应先沿用最近一次有效占位")
    events.append(mk(6, "短时停发 robot_position", short_timeout))

    def timeout_expired(s: GameState):
        cprint(YELLOW, f"[{s.present_time:.0f}s] 持续停发超过保持窗口，应视为控制区无人")
    events.append(mk(15, "超窗失效", timeout_expired))

    def out_of_bounds(s: GameState):
        s.publish_robot_position = True
        set_control_area_out_of_bounds(s)
        cprint(RED, f"[{s.present_time:.0f}s] 发布越界 robot_position，应等价于控制区无人")
    events.append(mk(18, "越界 robot_position", out_of_bounds))

    def nan_inf(s: GameState):
        set_control_area_nan_inf(s)
        cprint(RED, f"[{s.present_time:.0f}s] 发布 NaN/Inf robot_position，应保守降级且不崩溃")
    events.append(mk(22, "NaN/Inf robot_position", nan_inf))

    def recover(s: GameState):
        set_control_area_occupied(s, True)
        cprint(GREEN, f"[{s.present_time:.0f}s] 恢复有效 robot_position，应恢复 Chase")
    events.append(mk(26, "恢复有效 robot_position", recover))

    def enemy_take_control(s: GameState):
        s.central_point_state = 2
        cprint(RED, f"[{s.present_time:.0f}s] 敌方占点，进入 AMR 结束场景")
    events.append(mk(34, "敌方占点", enemy_take_control))

    def stop(s: GameState):
        s.game_progress = GameState.CALCULATION
        cprint(GREEN, f"[{s.present_time:.0f}s] 测试结束")
    events.append(mk(40, "结束", stop))
    return events


def build_enemy_revive_invincible_dirty_events(state: GameState):
    events = []

    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def start(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.referee_online = True
        s.central_point_state = 1
        s.robot_x, s.robot_y = 0.85, 1.05
        s.bullet_17mm = 220
        s.publish_robot_position = True
        s.publish_track = True
        set_control_area_occupied(s, False)
        set_track_target(s, 1, 50.15, 50.15)
        cprint(GREEN, f"[{s.present_time:.0f}s] 开始复活无敌 dirty 对照测试")
    events.append(mk(0, "开始", start))

    def freshly_revived_target(s: GameState):
        s.central_point_state = 1
        set_track_target(s, 1, 50.15, 50.15)
        cprint(YELLOW, f"[{s.present_time:.0f}s] 目标位于 freshly-resurrected 启发区")
    events.append(mk(2, "freshly resurrected 目标", freshly_revived_target))

    def attackable_target(s: GameState):
        set_track_target(s, 1, 2.6, 6.2)
        cprint(CYAN, f"[{s.present_time:.0f}s] 目标离开复活无敌区，应恢复 TE/P")
    events.append(mk(10, "目标离开复活无敌区", attackable_target))

    def nan_track(s: GameState):
        set_track_target(s, 1, math.nan, math.inf)
        cprint(RED, f"[{s.present_time:.0f}s] 发布 NaN/Inf track，应保守降级且不进入 TE/P")
    events.append(mk(13, "NaN/Inf track", nan_track))

    def recover_track(s: GameState):
        set_track_target(s, 1, 2.7, 6.0)
        cprint(GREEN, f"[{s.present_time:.0f}s] 恢复有效 track，应恢复 TE/P")
    events.append(mk(16, "恢复有效 track", recover_track))

    def stop(s: GameState):
        s.game_progress = GameState.CALCULATION
        cprint(GREEN, f"[{s.present_time:.0f}s] 测试结束")
    events.append(mk(20, "结束", stop))
    return events


def build_rulebook_full_chain_dirty_events(state: GameState):
    events = []

    def mk(t, desc, fn):
        return MatchEvent(t, desc, fn)

    def start_battle(s: GameState):
        s.game_progress = GameState.IN_BATTLE
        s.referee_online = True
        s.central_point_state = 0
        s.remain_hp = 400
        s.robot_hp['red_7'] = 400
        s.bullet_17mm = 320
        s.robot_x, s.robot_y = 0.85, 1.05
        s.publish_robot_position = True
        s.publish_track = True
        set_control_area_occupied(s, False)
        set_track_target(s, 0, 5.0, 2.0, tracking=False)
        cprint(GREEN, f"\n{'='*64}")
        cprint(GREEN, f"  [Dirty 全链路开始] 规则链路 + 脏环境注入")
        cprint(GREEN, f"{'='*64}\n")
    events.append(mk(0, "比赛开始（dirty full chain）", start_battle))

    def mode_gma(s: GameState):
        s.central_point_state = 0
        s.robot_x, s.robot_y = 0.85, 1.05
        set_control_area_occupied(s, False)
        set_track_target(s, 0, 5.0, 2.0, tracking=False)
        cprint(CYAN, f"[{s.present_time:.0f}s] clean 基线：GMA")
    events.append(mk(20, "基线 GMA", mode_gma))

    def mode_pmr(s: GameState):
        s.central_point_state = 1
        set_control_area_occupied(s, False)
        set_track_target(s, 0, 5.0, 2.0, tracking=False)
        cprint(CYAN, f"[{s.present_time:.0f}s] clean 基线：PMR")
    events.append(mk(45, "基线 PMR", mode_pmr))

    def mode_chase(s: GameState):
        s.central_point_state = 1
        s.robot_x, s.robot_y = 0.85, 1.05
        s.bullet_17mm = 280
        set_control_area_occupied(s, True)
        set_track_target(s, 3, 2.6, 4.2)
        cprint(RED, f"[{s.present_time:.0f}s] clean 基线：C")
    events.append(mk(70, "基线 Chase", mode_chase))

    def ctrl_timeout_1(s: GameState):
        s.publish_robot_position = False
        cprint(YELLOW, f"[{s.present_time:.0f}s] dirty#1 停发 /robot_position，先观察保持窗口")
    events.append(mk(84, "dirty#1 控制区短时超时", ctrl_timeout_1))

    def ctrl_timeout_expired_1(s: GameState):
        cprint(YELLOW, f"[{s.present_time:.0f}s] dirty#1 已超过保持窗口，应退出 Chase")
    events.append(mk(92, "dirty#1 控制区超窗", ctrl_timeout_expired_1))

    def ctrl_out_of_bounds_1(s: GameState):
        s.publish_robot_position = True
        set_control_area_out_of_bounds(s)
        cprint(RED, f"[{s.present_time:.0f}s] dirty#1 越界 robot_position")
    events.append(mk(100, "dirty#1 越界 robot_position", ctrl_out_of_bounds_1))

    def ctrl_nan_inf_1(s: GameState):
        set_control_area_nan_inf(s)
        cprint(RED, f"[{s.present_time:.0f}s] dirty#1 NaN/Inf robot_position")
    events.append(mk(108, "dirty#1 NaN/Inf robot_position", ctrl_nan_inf_1))

    def ctrl_restore_1(s: GameState):
        set_control_area_occupied(s, True)
        set_track_target(s, 3, 2.6, 4.2)
        cprint(GREEN, f"[{s.present_time:.0f}s] dirty#1 恢复有效控制区数据")
    events.append(mk(116, "dirty#1 恢复控制区数据", ctrl_restore_1))

    def revive_invincible_1(s: GameState):
        s.central_point_state = 1
        s.robot_x, s.robot_y = 0.85, 1.05
        set_control_area_occupied(s, True)
        set_track_target(s, 1, 50.15, 50.15)
        cprint(YELLOW, f"[{s.present_time:.0f}s] dirty#1 freshly resurrected 目标，应抑制 TE/P")
    events.append(mk(128, "dirty#1 复活无敌窗口", revive_invincible_1))

    def revive_attackable_1(s: GameState):
        set_track_target(s, 1, 2.6, 6.2)
        cprint(CYAN, f"[{s.present_time:.0f}s] dirty#1 目标恢复可攻击")
    events.append(mk(136, "dirty#1 可攻击目标恢复", revive_attackable_1))

    def hp_urgent_1(s: GameState):
        s.remain_hp = 40
        s.robot_hp['red_7'] = 40
        cprint(YELLOW, f"[{s.present_time:.0f}s] dirty#1 低血优先级生效，应进入 GHRA")
    events.append(mk(148, "dirty#1 低血", hp_urgent_1))

    def referee_offline_1(s: GameState):
        s.referee_online = False
        cprint(RED, f"[{s.present_time:.0f}s] dirty#1 裁判断联，应覆盖其它条件进入离线保守模式")
    events.append(mk(156, "dirty#1 裁判断联", referee_offline_1))

    def recover_1(s: GameState):
        s.referee_online = True
        s.remain_hp = 400
        s.robot_hp['red_7'] = 400
        s.central_point_state = 1
        s.robot_x, s.robot_y = 0.85, 1.05
        set_control_area_occupied(s, True)
        set_track_target(s, 3, 2.6, 4.2)
        cprint(GREEN, f"[{s.present_time:.0f}s] dirty#1 恢复后回到正常追击基线")
    events.append(mk(166, "dirty#1 恢复基线", recover_1))

    def ctrl_timeout_2(s: GameState):
        s.publish_robot_position = False
        cprint(YELLOW, f"[{s.present_time:.0f}s] dirty#2 再次停发 /robot_position，验证优先级不漂移")
    events.append(mk(178, "dirty#2 控制区短时超时", ctrl_timeout_2))

    def ctrl_timeout_expired_2(s: GameState):
        cprint(YELLOW, f"[{s.present_time:.0f}s] dirty#2 超过保持窗口，应再次退出 Chase")
    events.append(mk(186, "dirty#2 控制区超窗", ctrl_timeout_expired_2))

    def mode_unchase(s: GameState):
        s.publish_robot_position = True
        s.central_point_state = 1
        s.robot_x, s.robot_y = 2.846, 3.758
        s.bullet_17mm = 260
        set_control_area_occupied(s, True)
        set_track_target(s, 3, 2.7, 4.1)
        cprint(YELLOW, f"[{s.present_time:.0f}s] dirty#2 恢复有效数据，但自己处于追击受限区，应进入 UC")
    events.append(mk(194, "dirty#2 UnChase", mode_unchase))

    def revive_invincible_2(s: GameState):
        s.robot_x, s.robot_y = 0.85, 1.05
        s.central_point_state = 1
        set_control_area_occupied(s, True)
        set_track_target(s, 1, 50.15, 50.15)
        cprint(YELLOW, f"[{s.present_time:.0f}s] dirty#2 freshly resurrected 目标重复出现")
    events.append(mk(202, "dirty#2 复活无敌窗口", revive_invincible_2))

    def revive_attackable_2(s: GameState):
        set_track_target(s, 1, 2.6, 6.2)
        cprint(CYAN, f"[{s.present_time:.0f}s] dirty#2 目标恢复可攻击")
    events.append(mk(210, "dirty#2 可攻击目标恢复", revive_attackable_2))

    def hp_urgent_2(s: GameState):
        s.remain_hp = 45
        s.robot_hp['red_7'] = 45
        cprint(YELLOW, f"[{s.present_time:.0f}s] dirty#2 再次低血，应仍优先 GHRA")
    events.append(mk(220, "dirty#2 低血", hp_urgent_2))

    def referee_offline_2(s: GameState):
        s.referee_online = False
        cprint(RED, f"[{s.present_time:.0f}s] dirty#2 再次裁判断联，应仍优先 ABH/Stop")
    events.append(mk(228, "dirty#2 裁判断联", referee_offline_2))

    def recover_amr(s: GameState):
        s.referee_online = True
        s.remain_hp = 400
        s.robot_hp['red_7'] = 400
        s.central_point_state = 2
        s.robot_x, s.robot_y = 0.85, 1.05
        set_control_area_occupied(s, False)
        set_track_target(s, 0, 5.0, 2.0, tracking=False)
        cprint(RED, f"[{s.present_time:.0f}s] 恢复后切入 AMR")
    events.append(mk(238, "恢复后进入 AMR", recover_amr))

    def attack_phase(s: GameState):
        s.central_point_state = 2
        set_track_target(s, 3, 2.7, 4.0)
        cprint(RED, f"[{s.present_time:.0f}s] AMR 下重新获取目标")
    events.append(mk(252, "AMR 目标恢复", attack_phase))

    def end_battle(s: GameState):
        s.game_progress = GameState.CALCULATION
        s.referee_online = True
        cprint(GREEN, f"\n{'='*64}")
        cprint(GREEN, f"  [比赛结束] CALCULATION，应出现 CSG")
        cprint(GREEN, f"{'='*64}\n")
    events.append(mk(300, "比赛结束", end_battle))

    def offline_after_battle(s: GameState):
        s.referee_online = False
        cprint(RED, f"[{s.present_time:.0f}s] 非战斗阶段裁判断联，应出现 AS")
    events.append(mk(306, "赛后裁判断联", offline_after_battle))

    def tail(s: GameState):
        s.referee_online = True
        cprint(CYAN, f"[{s.present_time:.0f}s] 场景收尾")
    events.append(mk(312, "收尾", tail))

    return events


# ============================================================
# 发布节点
# ============================================================
class MatchSimulator:

    def __init__(self, scenario: str = 'full_match', speed: float = 1.0, external_tf: bool = False):
        rospy.init_node('match_simulator', anonymous=False)
        self.speed   = speed
        self.state   = GameState()
        self.running = True
        self.external_tf = external_tf
        self.connection_warmup_timeout_sec = rospy.get_param('~connection_warmup_timeout_sec', 15.0)
        self.publish_failure_warnings = set()
        self.last_manual_power_limit_state = None
        self.last_manual_log_time = rospy.Time(0)
        self.cap_log_path = os.environ.get("CAP_LOG", "")

        # TF 广播（若使用外部 fake_tf_publisher，则由其独占 TF）
        self.tf_broadcaster = None
        self.tf_dyn         = None
        if not self.external_tf:
            self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
            self.tf_dyn         = tf2_ros.TransformBroadcaster()

        # --- 发布者 ---
        self.pub_dbus    = rospy.Publisher('/rm_ecat_hw/dbus', DbusData, queue_size=1)
        self.pub_game    = rospy.Publisher('/rm_referee/game_status', GameStatus, queue_size=1)
        self.pub_robot   = rospy.Publisher('/rm_referee/game_robot_status', GameRobotStatus, queue_size=1)
        self.pub_hp      = rospy.Publisher('/rm_referee/game_robot_hp', GameRobotHp, queue_size=1)
        self.pub_event   = rospy.Publisher('/rm_referee/event_data', EventData, queue_size=1)
        self.pub_bullet  = rospy.Publisher('/rm_referee/bullet_allowance_data', BulletAllowance, queue_size=1)
        self.pub_rfid    = rospy.Publisher('/rm_referee/rfid_status_data', RfidStatus, queue_size=1)
        self.pub_track   = rospy.Publisher('/track', TrackData, queue_size=1)
        self.pub_sentry  = rospy.Publisher('/rm_referee/sentry_info', SentryInfo, queue_size=1)
        self.pub_buff    = rospy.Publisher('/rm_referee/robot_buff', Buff, queue_size=1)
        self.pub_radar   = rospy.Publisher('/rm_referee/radar_to_sentry', RadarToSentry, queue_size=1)
        self.pub_odom    = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.pub_power   = rospy.Publisher('/rm_referee/power_heat_data', PowerHeatData, queue_size=1)
        self.pub_capacity = rospy.Publisher('/rm_referee/power_management/sample_and_status',
                                            PowerManagementSampleAndStatusData, queue_size=1)
        self.pub_robot_position = rospy.Publisher('/robot_position', RobotsPositionData, queue_size=1)
        self.pub_fake_pose = rospy.Publisher('/fake_robot_pose', PoseStamped, queue_size=1)

        # --- 订阅监控话题 ---
        self.sub_log     = rospy.Subscriber('/behavior_tree/log', String, self._on_bt_log)
        self.sub_cmd     = rospy.Subscriber('/sentry_cmd', SentryCmd, self._on_sentry_cmd)
        self.sub_state   = rospy.Subscriber('/custom_info', String, self._on_custom_info)
        self.sub_manual  = rospy.Subscriber('/manual_to_referee', ManualToReferee, self._on_manual_to_referee)

        # 选择场景
        if scenario == 'hp_urgent':
            self.events = build_hp_urgent_events(self.state)
        elif scenario == 'rulebook_full_chain':
            self.events = build_rulebook_full_chain_events(self.state)
        elif scenario == 'rulebook_full_chain_dirty':
            self.events = build_rulebook_full_chain_dirty_events(self.state)
        elif scenario == 'chase_gate_020b':
            self.events = build_chase_gate_020b_events(self.state)
        elif scenario == 'control_area_dirty_gate':
            self.events = build_control_area_dirty_gate_events(self.state)
        elif scenario in ('posture_transitions', 'capacitor_middle_area_demo', 'capacitor_middle_area_disabled'):
            self.events = build_posture_transitions_events(self.state)
        elif scenario == 'referee_offline':
            self.events = build_referee_offline_events(self.state)
        elif scenario == 'attack_phase':
            self.events = build_attack_phase_events(self.state)
        elif scenario == 'no_bullets':
            self.events = build_no_bullets_events(self.state)
        elif scenario == 'enemy_invincible_transition':
            self.events = build_enemy_invincible_transition_events(self.state)
        elif scenario in ('enemy_revive_invincible_dirty_on', 'enemy_revive_invincible_dirty_off'):
            self.events = build_enemy_revive_invincible_dirty_events(self.state)
        elif scenario == 'weak_rfid_jitter':
            self.events = build_weak_rfid_jitter_events(self.state)
        elif scenario == 'weak_localization_bias_supply':
            self.events = build_weak_localization_bias_supply_events(self.state)
        elif scenario == 'conflict_chaos':
            self.events = build_conflict_chaos_events(self.state)
        elif scenario == 'death_revive':
            self.events = build_weak_rfid_jitter_events(self.state)
        else:
            self.events = build_full_match_events(self.state)

        # 按时间排序
        self.events.sort(key=lambda e: e.present_time)

        rospy.loginfo(f"[Simulator] 场景={scenario}  加速比={speed}x  事件数={len(self.events)}  external_tf={self.external_tf}")

    # --------------------------------------------------------
    # 静态 TF：odom -> base_link 固定偏移
    # --------------------------------------------------------
    def _publish_static_tf(self):
        if self.tf_broadcaster is None:
            return
        static_tf                          = TransformStamped()
        static_tf.header.stamp             = rospy.Time.now()
        static_tf.header.frame_id          = "odom"
        static_tf.child_frame_id           = "base_link"
        static_tf.transform.translation.x  = 0.0
        static_tf.transform.translation.y  = 0.0
        static_tf.transform.translation.z  = 0.0
        static_tf.transform.rotation.w     = 1.0
        self.tf_broadcaster.sendTransform(static_tf)

    # --------------------------------------------------------
    # 动态 TF：map -> odom (跟随机器人坐标)
    # --------------------------------------------------------
    def _publish_dynamic_tf(self):
        if self.tf_dyn is None:
            return
        t = TransformStamped()
        t.header.stamp          = rospy.Time.now()
        t.header.frame_id       = "map"
        t.child_frame_id        = "odom"
        t.transform.translation.x = self.state.robot_x
        t.transform.translation.y = self.state.robot_y
        t.transform.translation.z = 0.0
        # yaw → quaternion
        cy = math.cos(self.state.robot_yaw * 0.5)
        sy = math.sin(self.state.robot_yaw * 0.5)
        t.transform.rotation.w = cy
        t.transform.rotation.z = sy
        self.tf_dyn.sendTransform(t)

    def _publish_fake_robot_pose(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.state.robot_x
        msg.pose.position.y = self.state.robot_y
        msg.pose.position.z = 0.0
        cy = math.cos(self.state.robot_yaw * 0.5)
        sy = math.sin(self.state.robot_yaw * 0.5)
        msg.pose.orientation.w = cy
        msg.pose.orientation.z = sy
        self.pub_fake_pose.publish(msg)

    # --------------------------------------------------------
    # 消息构造
    # --------------------------------------------------------
    def _build_dbus(self) -> DbusData:
        msg = DbusData()
        if self.state.rc_online:
            msg.s_r   = self.state.rc_s_r   # UP=1=auto
            msg.stamp = rospy.Time.now()
        else:
            # 超时（时间戳故意设旧，让 isRemoteControlTurnOn 返回 FAILURE）
            msg.s_r   = DbusData.MID
            msg.stamp = rospy.Time.now() - rospy.Duration(2.0)
        return msg

    def _build_game_status(self) -> GameStatus:
        msg = GameStatus()
        msg.game_progress     = self.state.game_progress
        msg.stage_remain_time = max(0, int(300 - self.state.present_time))
        return msg

    def _build_robot_status(self) -> GameRobotStatus:
        msg = GameRobotStatus()
        msg.robot_id  = self.state.robot_id
        msg.remain_hp = self.state.remain_hp
        msg.max_hp    = self.state.max_hp
        return msg

    def _build_game_robot_hp(self) -> GameRobotHp:
        msg = GameRobotHp()
        # 2026 协议 0x0003 仅同步己方（ally）血量；仿真默认用 red_* 映射到 ally_*。
        msg.ally_1_robot_hp = self.state.robot_hp.get('red_1', 0)
        msg.ally_2_robot_hp = self.state.robot_hp.get('red_2', 0)
        msg.ally_3_robot_hp = self.state.robot_hp.get('red_3', 0)
        msg.ally_4_robot_hp = self.state.robot_hp.get('red_4', 0)
        msg.ally_7_robot_hp = self.state.robot_hp.get('red_7', 0)
        msg.ally_outpost_hp = self.state.robot_hp.get('red_outpost', 0)
        msg.ally_base_hp = self.state.robot_hp.get('red_base', 0)
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

    def _build_robots_position(self) -> RobotsPositionData:
        msg = RobotsPositionData()
        msg.hero_x = self.state.hero_x
        msg.hero_y = self.state.hero_y
        msg.engineer_x = self.state.engineer_x
        msg.engineer_y = self.state.engineer_y
        msg.standard_3_x = self.state.standard_3_x
        msg.standard_3_y = self.state.standard_3_y
        msg.standard_4_x = self.state.standard_4_x
        msg.standard_4_y = self.state.standard_4_y
        msg.stamp = rospy.Time.now()
        return msg

    def _build_track(self) -> TrackData:
        msg = TrackData()
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.id              = self.state.track_id
        msg.tracking        = (self.state.track_id != 0)
        msg.position.x      = self.state.track_pos_x
        msg.position.y      = self.state.track_pos_y
        msg.position.z      = 0.1
        msg.velocity.x      = self.state.track_vx
        msg.velocity.y      = self.state.track_vy
        return msg

    def _build_capacity(self) -> PowerManagementSampleAndStatusData:
        msg = PowerManagementSampleAndStatusData()
        msg.chassis_power = 80.0
        msg.chassis_expect_power = 120.0
        msg.capacity_recent_charge_power = self.state.capacity_recent_charge_power
        msg.capacity_remain_charge = self.state.capacity_remain_charge
        msg.capacity_discharge_power = self.state.capacity_discharge_power
        msg.state_machine_running_state = self.state.capacity_state_machine_running_state
        msg.power_management_protection_info = self.state.power_management_protection_info
        msg.power_management_topology = self.state.power_management_topology
        msg.stamp = rospy.Time.now()
        return msg

    def _build_radar(self) -> RadarToSentry:
        msg = RadarToSentry()
        msg.robot_ID   = self.state.radar_robot_id
        msg.position_x = self.state.radar_pos_x
        msg.position_y = self.state.radar_pos_y
        return msg

    def _build_odom(self) -> Odometry:
        msg = Odometry()
        msg.header.stamp          = rospy.Time.now()
        msg.header.frame_id       = "odom"
        msg.child_frame_id        = "base_link"
        msg.pose.pose.position.x  = 0.0
        msg.pose.pose.position.y  = 0.0
        msg.pose.pose.orientation.w = 1.0
        return msg

    # --------------------------------------------------------
    # 回调：订阅行为树输出
    # --------------------------------------------------------
    def _on_bt_log(self, msg: String):
        text = msg.data
        # 只打印包含关键模式变化的日志（避免刷屏）
        keywords = ['GetChassisDecisions', 'GetGimbalDecisions', 'GetShooterDecisions',
                    'ReviveIfDead', 'FAILURE', 'OutputRightSwitchState']
        if any(k in text for k in keywords):
            cprint(BLUE, f"  [BT_LOG] {text}")

    def _on_sentry_cmd(self, msg: SentryCmd):
        posture_map = {0: '无', 1: '攻击', 2: '防守', 3: '移动'}
        posture = posture_map.get(msg.posture_cmd, '?')
        revive  = '请求复活' if msg.confirm_respawn else ''
        cprint(MAGENTA, f"  [SENTRY_CMD] posture={posture}({msg.posture_cmd}) {revive}")

    def _on_custom_info(self, msg: String):
        cprint(CYAN, f"  [CUSTOM_INFO] {msg.data}")

    def _on_manual_to_referee(self, msg: ManualToReferee):
        self.state.manual_power_limit_state = msg.power_limit_state
        now = rospy.Time.now()
        if (self.last_manual_power_limit_state == msg.power_limit_state and
                now - self.last_manual_log_time < rospy.Duration(0.5)):
            return
        self.last_manual_power_limit_state = msg.power_limit_state
        self.last_manual_log_time = now
        line = f"[CAP] ManualToReferee time={self.state.present_time:.2f} state={msg.power_limit_state}\n"
        if self.cap_log_path:
            with open(self.cap_log_path, "a", encoding="utf-8") as f:
                f.write(line)
        else:
            print(line, end="", flush=True)

    def _warn_publish_failure(self, key: str, exc: Exception):
        if key in self.publish_failure_warnings:
            return
        self.publish_failure_warnings.add(key)
        cprint(YELLOW, f"[Simulator] {key} 发布失败，退化为无数据/超时路径: {exc}")

    def _required_connection_status(self):
        status = {
            '/rm_referee/game_status': self.pub_game.get_num_connections(),
            '/rm_referee/game_robot_status': self.pub_robot.get_num_connections(),
            '/track': self.pub_track.get_num_connections(),
            '/rm_referee/power_management/sample_and_status': self.pub_capacity.get_num_connections(),
        }
        if self.external_tf:
            status['/fake_robot_pose'] = self.pub_fake_pose.get_num_connections()
        return status

    def _wait_for_required_connections(self):
        deadline = rospy.Time.now() + rospy.Duration(self.connection_warmup_timeout_sec)
        rate = rospy.Rate(50)
        last_report_sec = -1
        while not rospy.is_shutdown():
            self._publish_all()
            status = self._required_connection_status()
            if all(count > 0 for count in status.values()):
                cprint(GREEN, f"[Simulator] 关键连接已就绪: {status}")
                return

            now = rospy.Time.now()
            elapsed_sec = int((deadline - now).to_sec())
            if elapsed_sec != last_report_sec:
                last_report_sec = elapsed_sec
                cprint(YELLOW, f"[Simulator] 等待关键连接: {status}")
            if now >= deadline:
                missing = ', '.join(f"{topic}={count}" for topic, count in status.items() if count <= 0)
                raise RuntimeError(f"关键连接未就绪: {missing}")
            rate.sleep()

    def _wait_for_manual_to_referee(self):
        try:
            msg = rospy.wait_for_message('/manual_to_referee', ManualToReferee,
                                         timeout=self.connection_warmup_timeout_sec)
            self._on_manual_to_referee(msg)
        except rospy.ROSException as exc:
            raise RuntimeError(f"/manual_to_referee 未就绪: {exc}")

    # --------------------------------------------------------
    # 主循环
    # --------------------------------------------------------
    def run(self):
        if not self.external_tf:
            self._publish_static_tf()
        rate = rospy.Rate(50)   # 50Hz 发布频率

        cprint(BOLD, "\n[Simulator] 开始发布仿真数据，等待 rm_behavior_tree 节点启动...\n")

        # 在开始计时前持续 warm-up，直到关键订阅连接真正建立。
        try:
            self._wait_for_required_connections()
            self._wait_for_manual_to_referee()
        except RuntimeError as exc:
            rospy.logerr(str(exc))
            raise

        # 记录比赛开始的实际时间
        sim_start = rospy.Time.now().to_sec()

        cprint(BOLD, "[Simulator] 开始比赛仿真...\n")
        cprint(BOLD, f"{'─'*70}")
        cprint(BOLD, f"{'时间(s)':>8} | {'底盘模式':^20} | {'云台模式':^16} | {'射击模式':^8} | {'区域':^18}")
        cprint(BOLD, f"{'─'*70}")

        last_print_time  = -1.0
        last_chassis     = ""
        last_gimbal      = ""

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            elapsed = (now - sim_start) * self.speed   # 支持加速

            # 更新 present_time
            if self.state.is_in_battle:
                self.state.present_time     = elapsed
                self.state.stage_remain_time = max(0.0, 300.0 - elapsed)

            # 触发到期事件
            for ev in self.events:
                if not ev.triggered and elapsed >= ev.present_time:
                    ev.triggered = True
                    cprint(YELLOW, f"\n>>> [{elapsed:.1f}s] 事件: {ev.description}")
                    ev.apply_fn(self.state)

            # 发布所有话题
            self._publish_all()

            # 比赛结束后延迟30s退出
            if self.state.game_progress == GameState.CALCULATION:
                if elapsed > self.state.present_time + 30:
                    break

            rate.sleep()

        cprint(GREEN, "\n[Simulator] 仿真结束。")

    def _publish_all(self):
        now = rospy.Time.now()
        if self.external_tf:
            self._publish_fake_robot_pose()
        else:
            self._publish_dynamic_tf()

        # 遥控器
        self.pub_dbus.publish(self._build_dbus())

        # 裁判系统在线：发布完整裁判数据
        if self.state.referee_online:
            self.pub_game.publish(self._build_game_status())
            self.pub_robot.publish(self._build_robot_status())
            self.pub_hp.publish(self._build_game_robot_hp())
            self.pub_event.publish(self._build_event_data())
            self.pub_bullet.publish(self._build_bullet_allowance())
            self.pub_rfid.publish(self._build_rfid())
            if self.state.publish_robot_position:
                try:
                    self.pub_robot_position.publish(self._build_robots_position())
                except Exception as exc:
                    self._warn_publish_failure('robot_position', exc)
            self.pub_buff.publish(Buff())
            sentry_info = SentryInfo()
            self.pub_sentry.publish(sentry_info)
            # PowerHeatData — 关键！设置 stamp 使 referee_is_online_ = true
            pwr = PowerHeatData()
            pwr.stamp = rospy.Time.now()
            pwr.chassis_power_buffer = 60
            self.pub_power.publish(pwr)
            self.pub_capacity.publish(self._build_capacity())
        else:
            # 裁判系统离线：持续发布“过期” PowerHeatData，驱动 referee_is_online_=false。
            # 仅停发其余裁判数据，保持最近一次 game_status 作为离线时判定依据。
            pwr = PowerHeatData()
            pwr.stamp = rospy.Time.now() - rospy.Duration(2.0)
            pwr.chassis_power_buffer = 0
            self.pub_power.publish(pwr)
            stale_capacity = self._build_capacity()
            stale_capacity.stamp = rospy.Time.now() - rospy.Duration(2.0)
            self.pub_capacity.publish(stale_capacity)

        # 目标跟踪（始终发布，allow id=0 表示无目标）
        if self.state.publish_track:
            try:
                self.pub_track.publish(self._build_track())
            except Exception as exc:
                self._warn_publish_failure('track', exc)

        # 雷达数据
        try:
            self.pub_radar.publish(self._build_radar())
        except Exception:
            pass

        # 里程计
        self.pub_odom.publish(self._build_odom())


# ============================================================
# 入口
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="RMUC 2026 比赛仿真驱动")
    parser.add_argument('--scenario', default='full_match',
                        choices=['full_match', 'rulebook_full_chain', 'rulebook_full_chain_dirty',
                                 'chase_gate_020b', 'control_area_dirty_gate', 'posture_transitions',
                                 'capacitor_middle_area_demo', 'capacitor_middle_area_disabled',
                                 'hp_urgent', 'death_revive', 'referee_offline', 'attack_phase', 'no_bullets',
                                 'enemy_invincible_transition', 'enemy_revive_invincible_dirty_on',
                                 'enemy_revive_invincible_dirty_off', 'weak_rfid_jitter',
                                 'weak_localization_bias_supply', 'conflict_chaos'],
                        help='选择仿真场景')
    parser.add_argument('--speed', type=float, default=3.0,
                        help='仿真加速倍率（默认3x，即100s仿真实际33s）')
    parser.add_argument('--external-tf', action='store_true',
                        help='不直接发布TF，仅发布 /fake_robot_pose 给 fake_tf_publisher 使用')
    # ROS 会向 argv 注入额外参数，需要过滤
    args, _ = parser.parse_known_args()

    sim = MatchSimulator(scenario=args.scenario, speed=args.speed, external_tf=args.external_tf)
    sim.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
