#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fake_tf_publisher.py
发布仿真所需的完整 TF 树，使 rm_behavior_tree 能正常执行 TF 查询。

发布的 TF 变换:
  map        → odom            (动态，跟随 /odom 里程计)
  odom       → base_link       (固定原点)
  base_link  → camera_optical_frame   (前摄像头)
  base_link  → back_camera_optical_frame (后摄像头)

另外订阅 /fake_robot_pose 可动态更新机器人地图坐标。
"""

import math
import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry


def make_tf(parent: str, child: str,
            tx=0.0, ty=0.0, tz=0.0,
            roll=0.0, pitch=0.0, yaw=0.0) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp    = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id  = child
    t.transform.translation.x = tx
    t.transform.translation.y = ty
    t.transform.translation.z = tz
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    t.transform.rotation.w = cr * cp * cy + sr * sp * sy
    t.transform.rotation.x = sr * cp * cy - cr * sp * sy
    t.transform.rotation.y = cr * sp * cy + sr * cp * sy
    t.transform.rotation.z = cr * cp * sy - sr * sp * cy
    return t


class FakeTFPublisher:
    def __init__(self):
        rospy.init_node('fake_tf_publisher', anonymous=False)

        self.static_br  = tf2_ros.StaticTransformBroadcaster()
        self.dynamic_br = tf2_ros.TransformBroadcaster()

        # 机器人地图位置（可被外部更新）
        self.map_x   = 2.846   # 初始在 middle_area
        self.map_y   = 3.758
        self.map_yaw = 0.0

        # 订阅外部位姿更新
        rospy.Subscriber('/fake_robot_pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/odom', Odometry, self._odom_cb)

        # 发布静态 TF（不动的部分）
        self._publish_static_transforms()

        rospy.loginfo("[FakeTF] TF 仿真发布器启动，初始位置 (%.2f, %.2f)" % (self.map_x, self.map_y))

    def _publish_static_transforms(self):
        transforms = [
            # base_link → 前摄像头（正前方，稍微抬头）
            make_tf("base_link", "camera_optical_frame",
                    tx=0.20, ty=0.0, tz=0.10, pitch=-0.15),
            # base_link → 后摄像头（朝后，180度偏转）
            make_tf("base_link", "back_camera_optical_frame",
                    tx=-0.20, ty=0.0, tz=0.10, yaw=math.pi, pitch=-0.15),
            # base_link → lidar
            make_tf("base_link", "velodyne",
                    tx=0.0, ty=0.0, tz=0.25),
            # base_link → imu
            make_tf("base_link", "imu_link",
                    tx=0.0, ty=0.0, tz=0.0),
        ]
        self.static_br.sendTransform(transforms)
        rospy.loginfo("[FakeTF] 静态 TF 已发布 (%d 条)" % len(transforms))

    def _pose_cb(self, msg: PoseStamped):
        self.map_x   = msg.pose.position.x
        self.map_y   = msg.pose.position.y
        # 从四元数提取 yaw
        q = msg.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.map_yaw = math.atan2(siny, cosy)

    def _odom_cb(self, msg: Odometry):
        # odom 到 base_link 直接用消息里的 pose
        t = TransformStamped()
        t.header.stamp    = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id  = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation      = msg.pose.pose.orientation
        self.dynamic_br.sendTransform(t)

    def _publish_map_to_odom(self):
        """发布 map → odom，跟随模拟机器人位置"""
        t = make_tf("map", "odom",
                    tx=self.map_x, ty=self.map_y,
                    yaw=self.map_yaw)
        t.header.stamp = rospy.Time.now()
        self.dynamic_br.sendTransform(t)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self._publish_map_to_odom()
            rate.sleep()


if __name__ == '__main__':
    try:
        FakeTFPublisher().run()
    except rospy.ROSInterruptException:
        pass
