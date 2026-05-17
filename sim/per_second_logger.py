#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
per_second_logger.py — 从 BT 节点的日志文件中解析每秒行为状态

用法:
  python3 per_second_logger.py <bt_log_file> [duration_sec]

它会 tail -f 日志文件，解析 [BT] 行，每秒输出一行汇总。
不依赖 ROS topic (因为 bt_log_pub_ 在仿真中可能无消息)，
直接解析 BT 节点的 stdout/stderr 日志文件。
"""

import re
import sys
import time
import subprocess
import signal
import select
from collections import OrderedDict, Counter

LOG_RE = re.compile(
    r'\[BT\] (?P<node>\S+)\s+(?P<status>SUCCESS|FAILURE|RUNNING)'
    r'\s+C:(?P<chassis>\S+)\s+G:(?P<gimbal>\S+)\s+B:(?P<booster>\S+)'
    r'\s+pos=\((?P<px>[\-\d.]+),(?P<py>[\-\d.]+)\)(?:@(?P<area>\S+))?'
    r'\s+Enemy:(?P<enemy>\S+)'
    r'\s+Time:(?P<time>[\d.]+)'
    r'(?:\s+(?P<extra>.*))?'
)

CHASSIS_FULL = {
    'GMA': 'GotoMiddleArea', 'AMR': 'AttackMidRobot', 'PMR': 'ProtectMidRobot',
    'CSG': 'ChassisSlowGyro', 'AS': 'AbnormalStill', 'ABH': 'AbnormalBackHome',
    'GHRA': 'GotoHpReturnArea', 'C': 'Chase', 'UC': 'UnChase', 'AD': 'AvoidDrone',
}
GIMBAL_FULL = {
    'YSR': 'YawSlowRound', 'AS': 'AbnormalStill', 'LTF': 'LidarFront',
    'RSE': 'RoundSearch', 'FSE': 'FanSearch', 'IG': 'InverseGimbal', 'TE': 'TrackEnemy',
}
BOOSTER_FULL = {'S': 'Stop', 'R': 'Ready', 'P': 'Push'}


def parse_line(line):
    m = LOG_RE.search(line)
    if not m:
        return None
    return {
        'node': m.group('node'),
        'status': m.group('status'),
        'chassis': m.group('chassis'),
        'gimbal': m.group('gimbal'),
        'booster': m.group('booster'),
        'px': float(m.group('px')),
        'py': float(m.group('py')),
        'area': m.group('area') or '',
        'enemy': m.group('enemy'),
        'bt_time': float(m.group('time')),
        'extra': (m.group('extra') or '').strip(),
    }


def print_summary(records):
    if not records:
        print("\n[!] 没有采集到任何数据", flush=True)
        return
    sep = '═' * 80
    print(f"\n{sep}")
    print("  比赛仿真完成 — 模式分布统计")
    print(f"{sep}")
    total = max(len(records), 1)
    c_cnt = Counter(r['chassis'] for r in records.values() if r['chassis'] != '?')
    g_cnt = Counter(r['gimbal'] for r in records.values() if r['gimbal'] != '?')
    b_cnt = Counter(r['booster'] for r in records.values() if r['booster'] != '?')
    print("\n底盘模式占比:")
    for k, v in c_cnt.most_common():
        bar = '█' * int(30 * v / total)
        print(f"  {CHASSIS_FULL.get(k, k):<22} {bar:<30} {v:>4}s ({100 * v // total:>3}%)")
    print("\n云台模式占比:")
    for k, v in g_cnt.most_common():
        bar = '█' * int(30 * v / total)
        print(f"  {GIMBAL_FULL.get(k, k):<22} {bar:<30} {v:>4}s ({100 * v // total:>3}%)")
    print("\n射击模式占比:")
    for k, v in b_cnt.most_common():
        bar = '█' * int(30 * v / total)
        print(f"  {BOOSTER_FULL.get(k, k):<10} {bar:<30} {v:>4}s ({100 * v // total:>3}%)")
    print("\n底盘模式切换记录:")
    prev = None
    for sec, rec in records.items():
        c = rec['chassis']
        if c != prev and c != '?':
            print(f"  t={sec:>4}s  (BT:{rec['bt_time']:>6.1f}s)  → {CHASSIS_FULL.get(c, c):<22}  @{rec['area']}")
            prev = c
    print("\n云台模式切换记录:")
    prev = None
    for sec, rec in records.items():
        g = rec['gimbal']
        if g != prev and g != '?':
            print(f"  t={sec:>4}s  → {GIMBAL_FULL.get(g, g):<22}  @{rec['area']}")
            prev = g
    print(f"\n{'─' * 80}\n", flush=True)


def main():
    if len(sys.argv) < 2:
        print("用法: python3 per_second_logger.py <bt_log_file> [duration_sec]")
        sys.exit(1)

    log_file = sys.argv[1]
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 120

    records = OrderedDict()
    cur = {'chassis': '?', 'gimbal': '?', 'booster': '?',
           'area': '?', 'bt_time': 0.0, 'enemy': '?', 'extra': '',
           'px': 0.0, 'py': 0.0}
    fail_buf = []
    msg_count = 0
    last_log_update = time.time()
    idle_warned = False

    hdr = f"{'秒':>5} | {'底盘模式':<22} | {'云台模式':<16} | {'射击':>5} | {'区域':<26} | {'BT_Time':>8} | {'敌方':>6} | {'备注'}"
    sep = '─' * 115
    print(f"\n{sep}\n{hdr}\n{sep}", flush=True)

    proc = subprocess.Popen(
        ['tail', '-f', '-n', '+1', log_file],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        universal_newlines=True, bufsize=1
    )

    start = time.time()
    tick = 0
    last_sample_time = start

    def cleanup(signum=None, frame=None):
        proc.kill()
        print_summary(records)
        sys.exit(0)
    signal.signal(signal.SIGTERM, cleanup)
    signal.signal(signal.SIGINT, cleanup)

    try:
        while True:
            now = time.time()
            if now - start > duration:
                break

            while select.select([proc.stdout], [], [], 0.05)[0]:
                line = proc.stdout.readline()
                if not line:
                    break
                parsed = parse_line(line)
                if parsed:
                    msg_count += 1
                    last_log_update = time.time()
                    if idle_warned:
                        print("[INFO] BT 日志恢复更新", flush=True)
                        idle_warned = False
                    for k in ('chassis', 'gimbal', 'booster', 'area', 'bt_time',
                              'enemy', 'extra', 'px', 'py'):
                        cur[k] = parsed[k]
                    if parsed['status'] == 'FAILURE':
                        fail_buf.append(parsed['node'])

            now2 = time.time()
            remaining = duration - (now2 - start)
            should_warn_idle = (not records) or remaining > 5.0
            if now2 - last_log_update >= 5.0 and not idle_warned and should_warn_idle:
                print(f"[WARN] BT 日志已 {now2 - last_log_update:.1f}s 无新增，请检查节点是否卡住或启动失败", flush=True)
                idle_warned = True
            if now2 - last_sample_time >= 1.0:
                last_sample_time = now2
                rec = dict(cur)
                rec['fail_nodes'] = list(fail_buf)
                rec['msg_count'] = msg_count
                fail_buf.clear()
                msg_count = 0
                records[tick] = rec

                c_full = CHASSIS_FULL.get(rec['chassis'], rec['chassis'])
                g_full = GIMBAL_FULL.get(rec['gimbal'], rec['gimbal'])
                b_full = BOOSTER_FULL.get(rec['booster'], rec['booster'])
                fails = '/'.join(rec['fail_nodes'][:3]) if rec['fail_nodes'] else ''
                note = rec.get('extra', '') or fails

                print(f"{tick:>5} | {c_full:<22} | {g_full:<16} | {b_full:>5} | "
                      f"{rec['area']:<26} | {rec['bt_time']:>8.1f} | {rec['enemy']:>6} | {note}",
                      flush=True)
                tick += 1

    except KeyboardInterrupt:
        pass
    finally:
        proc.kill()
        proc.wait()

    print_summary(records)


if __name__ == '__main__':
    main()
