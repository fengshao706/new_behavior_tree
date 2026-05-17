#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fake_dynreconf_server.py
提供假的 dynamic_reconfigure 服务器，
使 BT 节点的 dynamic_reconfigure::Client 构造函数不再阻塞。

需要响应:
  /move_base_flex/GlobalPlanner/set_parameters
  /move_base_flex/set_parameters
"""

import rospy
from dynamic_reconfigure.srv import Reconfigure, ReconfigureResponse
from dynamic_reconfigure.msg import Config, BoolParameter, IntParameter, DoubleParameter, StrParameter


def handle_reconfigure(req):
    """处理 set_parameters 请求 — 直接返回请求的配置"""
    resp = ReconfigureResponse()
    resp.config = req.config
    return resp


def main():
    rospy.init_node('fake_dynreconf_server', anonymous=True)

    # 创建 dynamic_reconfigure 客户端需要的服务
    namespaces = [
        '/move_base_flex/GlobalPlanner',
        '/move_base_flex',
    ]

    services = []
    for ns in namespaces:
        # dynamic_reconfigure::Client 等待 <ns>/set_parameters 服务
        svc = rospy.Service(ns + '/set_parameters', Reconfigure, handle_reconfigure)
        services.append(svc)
        rospy.loginfo(f"Fake dynreconf server: {ns}/set_parameters")

        # 还需要发布 parameter_descriptions 和 parameter_updates 话题
        # dynamic_reconfigure::Client 在构造后会订阅这些来获取初始配置
        desc_pub = rospy.Publisher(ns + '/parameter_descriptions', Config, queue_size=1, latch=True)
        upd_pub = rospy.Publisher(ns + '/parameter_updates', Config, queue_size=1, latch=True)

        # 发布一个空的 latched 配置
        empty_cfg = Config()
        desc_pub.publish(empty_cfg)
        upd_pub.publish(empty_cfg)

    rospy.loginfo("Fake dynamic_reconfigure servers ready.")
    rospy.spin()


if __name__ == '__main__':
    main()
