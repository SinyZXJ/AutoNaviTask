#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# from _dbus_bindings import Int32
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger, TriggerResponse
import time
import math
import subprocess
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import os
class JackalMotionController(object):
    def __init__(self):
        rospy.init_node('jackal_motion_controller', anonymous=True)
        # 发布运动指令
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # 发布解锁桥梁命令（移除锥桶），发布 True 到 /cmd_open_bridge
        self.bridge_unlock_pub = rospy.Publisher('/cmd_open_bridge', Bool, queue_size=1)
        # 发布 state 更新（用于程序结束前将 state 设置为 4）
        self.state_pub = rospy.Publisher('/state', Int32, queue_size=1)

        self.state_sub = rospy.Subscriber('/state', Int32, self.state_callback)

        # 订阅扫描结果： error (Float32) 与 transform (Vector3)
        self.error_sub = rospy.Subscriber('/registration_error', Float32, self.error_callback)
        self.transform_sub = rospy.Subscriber('/registration_transform', Vector3, self.transform_callback)
        rospy.loginfo("1")
        # # 等待 /run_registration 服务，并建立客户端
        # rospy.wait_for_service('run_registration')
        # self.scan_service = rospy.ServiceProxy('run_registration', Trigger)

        self.current_error = None
        self.current_transform = None
        self.current_state = 3

        self.rate = rospy.Rate(1)
        # 状态机初始状态
        self.state = "initial"

    def error_callback(self, msg):
        self.current_error = msg.data

    def transform_callback(self, msg):
        self.current_transform = msg

    def state_callback(self, msg):
        #self.current_state = msg.data
        self.current_state = 3

    def move_distance(self, distance, speed=0.2):
        """
        移动指定距离（正为前进，负为后退）
        """
        twist = Twist()
        twist.linear.x = speed if distance > 0 else -speed
        duration = abs(distance) / speed
        t0 = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - t0).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())

    def move_correction(self, dx, dy, speed=1, angular_speed=0.2):
        """
        根据 dx, dy 的期望位移进行修正运动，
        机器人只能控制前进/后退和旋转，所以采用如下策略：
          1. 计算目标位移 r 和期望转向角 angle
          2. 先旋转 angle，然后前进 r，再旋转回 -angle
        """
        # 计算期望位移的极坐标表示
        r = math.sqrt(dx ** 2 + dy ** 2)
        if r < 1e-3:
            return  # 差值太小，无需运动
        angle = math.atan2(dy, dx)
        rospy.loginfo("move_correction: r = {:.4f}, angle = {:.4f}".format(r, angle))

        # 先旋转到目标方向
        self.rotate_angle(angle, angular_speed)
        # 前进期望距离
        self.move_distance(r, speed)
        # 再旋转回原始方向
        self.rotate_angle(-angle, angular_speed)

    def rotate_angle(self, angle, angular_speed=0.1):
        """
        旋转指定角度（弧度），正为逆时针，负为顺时针
        """
        twist = Twist()
        twist.angular.z = angular_speed if angle > 0 else -angular_speed
        duration = abs(angle) / angular_speed
        t0 = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - t0).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())

    def call_scan(self):
        """
        调用 /run_registration 服务进行一次扫描，并等待扫描结果更新
        """
        try:
            res = self.scan_service()
            rospy.loginfo("Scan response: {}".format(res.message))
        except rospy.ServiceException as e:
            rospy.logerr("Scan service call failed: {}".format(e))
        rospy.sleep(1.0)

    def run(self):
        """
        状态机控制流程：
         1. 初始状态（initial）：调用扫描服务；如果 error > 0.08，则前进 1m后停下，再次扫描；
            如果 error 在 [0.04, 0.08) 之间，进入 correction 状态；
            如果 error < 0.04，则进入 fine_correction 状态。
         2. correction 状态：进行修正运动。此处运动量为：
            - x、y 分别采用 transform_msg 的差值的一半（注意坐标转换：transform_msg.y → robot x，transform_msg.x → robot y）
            - 同时加入 yaw 修正，取 transform_msg.z 的 0.1 倍
            然后重新扫描；若 error 仍在 [0.04, 0.08) 内则重复此状态；
            若 error 升至 ≥0.08，则重新扫描，若依然 ≥0.08，则根据 transform_msg.y 判断前进/后退 0.3m后回到 initial 状态；
            若 error 降至 < 0.04，则进入 fine_correction 状态。
         3. fine_correction 状态：先按 transform_msg 全量修正（不做 yaw 转动），然后扫描；
            如果 error < 0.04 且 transform_msg.x 与 transform_msg.y 的绝对值均小于 0.2，则执行转动（根据 transform_msg.z，负顺时针，正逆时针），
            随后前进 1m，发布解锁桥梁命令（发布 True 到 /cmd_open_bridge），再前进 2m，最后进入静止状态（退出控制循环）。
            否则回到 initial 状态。
        """
        # 等待 state 主题值为 3（假设已订阅 /state 话题）
        rospy.loginfo("等待 state 为 3，当前 state: {}".format(self.current_state))
        print(os.path.dirname(__file__))
        while self.current_state != 3 and not rospy.is_shutdown():
            rospy.sleep(0.5)
        rospy.loginfo("state 为 3，开始执行程序。")


        # 启动外部脚本 TEST5.py
        rospy.loginfo("启动 TEST5.py...")
        test5_path = os.path.join(os.path.dirname(__file__), "TEST5.py")
        
        subprocess.Popen(["python3", test5_path])
        rospy.sleep(1.0)  # 可选：确保脚本启动

        # 在启动外部脚本后等待服务启动并建立服务代理
        rospy.wait_for_service('run_registration')
        self.scan_service = rospy.ServiceProxy('run_registration', Trigger)



        while not rospy.is_shutdown():
            rospy.loginfo("Calling scan service...")
            self.call_scan()
            if self.current_error is None or self.current_transform is None:
                rospy.logwarn("Scan data not available yet.")
                self.rate.sleep()
                continue

            rospy.loginfo("Current error: {:.4f}".format(self.current_error))
            # 初始状态
            if self.current_error>50:
                # self.move_correction(0.3, -0.5)
                self.move_distance(0.5)
            if self.state == "initial":
                if 50>self.current_error > 0.08:
                    rospy.loginfo("State initial: Error > 0.08, moving forward 0.5m.")
                    self.move_distance(0.5)
                    continue
                elif 0.04 <= self.current_error <= 0.08:
                    rospy.loginfo("State initial: Error in [0.04, 0.08), transitioning to correction state.")
                    self.state = "correction"
                elif self.current_error < 0.04:
                    rospy.loginfo("State initial: Error < 0.04, transitioning to fine_correction state.")
                    self.state = "fine_correction"

            # correction 状态：进行修正运动，运动量为 x、y 的 0.5 倍，并加入 yaw 的 0.1 倍
            if self.state == "correction":
                # 注意：transform_msg.y 对应 robot 的 x 轴，transform_msg.x 对应 robot 的 y 轴
                self.current_transform.y = max(-2, min(2, self.current_transform.y))
                self.current_transform.x = max(-2, min(2, self.current_transform.x))

                corr_x = 0.5 * self.current_transform.y
                corr_y = 0.5 * self.current_transform.x
                rospy.loginfo(
                    "State correction: Performing half correction: corr_x: {:.4f}, corr_y: {:.4f}".format(corr_x,
                                                                                                          corr_y))
                self.move_correction(corr_x, corr_y)
                # 加入 yaw 修正，取 transform_msg.z 的 0.1 倍
                # corr_yaw = 0.05 * self.current_transform.z
                # rospy.loginfo("State correction: Rotating by 0.05 factor: corr_yaw: {:.4f}".format(corr_yaw))
                # self.rotate_angle(corr_yaw)
                self.call_scan()
                if self.current_error is None or self.current_transform is None:
                    continue
                if 0.04 <= self.current_error <= 0.08:
                    rospy.loginfo("State correction: Error still in [0.04, 0.08), repeating correction.")
                    self.state = "correction"
                elif 50>self.current_error > 0.08:
                    rospy.loginfo("State correction: Error increased > 0.08, scanning again...")
                    self.call_scan()
                    if 50>self.current_error > 0.08:
                        if self.current_transform.y > 0:
                            rospy.loginfo("State correction: Error high, moving forward 0.3m.")
                            self.move_distance(0.3)
                        else:
                            rospy.loginfo("State correction: Error high, moving backward 0.3m.")
                            self.move_distance(-0.3)
                        self.state = "initial"
                    else:
                        self.state = "correction"
                elif self.current_error < 0.04:
                    rospy.loginfo("State correction: Error dropped below 0.04, transitioning to fine_correction state.")
                    self.state = "fine_correction"

            # fine_correction 状态：全量修正（不做 yaw 转动），然后检查是否需要旋转
            if self.state == "fine_correction":
                # self.current_transform.y = max(-1, min(1, self.current_transform.y))
                # self.current_transform.x = max(-1, min(1, self.current_transform.x))
                corr_x = self.current_transform.y  # robot x 轴修正
                corr_y = self.current_transform.x  # robot y 轴修正
                rospy.loginfo(
                    "State fine_correction: Performing full correction, corr_x: {:.4f}, corr_y: {:.4f}".format(corr_x,
                                                                                                               corr_y))
                self.move_correction(corr_x, corr_y)
                self.call_scan()
                if self.current_error is None or self.current_transform is None:
                    continue
                if self.current_error < 0.04 and abs(self.current_transform.x) < 0.3 :
                    rospy.loginfo("State fine_correction: Conditions met for rotation, angle: {:.4f}".format(
                        self.current_transform.z))
                    subprocess.call("rosnode kill $(rosnode list | grep pointcloud_registration_node)", shell=True)
                    self.rotate_angle(self.current_transform.z)
                    self.rotate_angle(-0.1)
                    rospy.loginfo("State fine_correction: Moving forward 2m after rotation.")
                    self.move_distance(2.0)

                    rospy.loginfo("State fine_correction: Publishing bridge unlock command.")
                    unlock_msg = Bool()
                    unlock_msg.data = True
                    self.bridge_unlock_pub.publish(unlock_msg)
                    rospy.loginfo("State fine_correction: Moving forward 2m after bridge unlock.")
                    self.move_distance(2.5,speed=3)
                    rospy.loginfo("Sequence complete, robot is now stationary.")
                    self.cmd_vel_pub.publish(Twist())
                    return  # 退出状态机，进入静止状态
                else:
                    rospy.loginfo("State fine_correction: Conditions not met for rotation, resetting to initial state.")
                    self.state = "initial"

            self.rate.sleep()


if __name__ == '__main__':

        try:
            controller = JackalMotionController()
            controller.run()
        except rospy.ROSInterruptException:
            pass
        finally:
            # 程序异常退出前也确保 state 被更新为 4
            try:
                state_msg = Int32()
                state_msg.data = 4
                controller.state_pub.publish(state_msg)
                rospy.loginfo("程序结束前，最终 state 更新为 4。")
            except:
                pass
