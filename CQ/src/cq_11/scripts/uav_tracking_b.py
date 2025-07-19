#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
import math

# === 控制器参数 ===
Kp = 0.6     # 比例增益
Ki = 0.05    # 积分增益
Kd = 0.2     # 微分增益

max_speed = 2        # 最大速度 m/s
max_acc = 1.0          # 最大加速度 m/s²
dt = 1.0 / 60.0        # 控制周期

# === 全局变量 ===
self_pose = PoseStamped()
target_pose = PoseStamped()
twist = TwistStamped()
find_cnt = 0
find_cnt_last = 0
not_find_time = 0
get_time = False

# PID 变量
error_last = [0.0, 0.0, 0.0]
integral = [0.0, 0.0, 0.0]
velocity_last = [0.0, 0.0, 0.0]

def saturate(val, limit):
    return max(-limit, min(val, limit))

def self_pose_callback(msg):
    global self_pose
    self_pose = msg

def target_pose_callback(msg):
    global target_pose, twist, find_cnt, get_time
    global error_last, integral, velocity_last

    target_pose = msg

    dx = target_pose.pose.position.x - self_pose.pose.position.x
    dy = target_pose.pose.position.y - self_pose.pose.position.y
    dz = target_pose.pose.position.z - self_pose.pose.position.z

    # PID 控制
    for i, err in enumerate([dx, dy, dz]):
        integral[i] += err * dt
        derivative = (err - error_last[i]) / dt
        v_target = Kp * err + Ki * integral[i] + Kd * derivative

        # 加速度限制
        dv = v_target - velocity_last[i]
        dv = saturate(dv, max_acc * dt)
        velocity_last[i] += dv
        velocity = saturate(velocity_last[i], max_speed)

        if i == 0:
            twist.twist.linear.x = velocity
        elif i == 1:
            twist.twist.linear.y = velocity
        else:
            twist.twist.linear.z = velocity

        error_last[i] = err

    twist.header.stamp = rospy.Time.now()

    find_cnt += 1
    get_time = False

if __name__ == "__main__":
    rospy.init_node("robust_tracking_node")

    # === 订阅目标与自身位置 ===
    rospy.Subscriber("iris_0/mavros/vision_pose/pose", PoseStamped, self_pose_callback)
    rospy.Subscriber("iris_1/mavros/vision_pose/pose", PoseStamped, target_pose_callback)

    # === 发布速度控制 ===
    vel_pub = rospy.Publisher("iris_0/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)

    # === 预发布 setpoint 防止切换失败 ===
    rospy.loginfo("Publishing pre-setpoints for OFFBOARD mode...")
    twist.twist.linear.z = 0.1
    for _ in range(100):
        twist.header.stamp = rospy.Time.now()
        vel_pub.publish(twist)
        rospy.sleep(0.02)

    # === 解锁与切换 OFFBOARD 模式 ===
    rospy.wait_for_service("iris_0/mavros/cmd/arming")
    arming = rospy.ServiceProxy("iris_0/mavros/cmd/arming", CommandBool)
    arm_resp = arming(True)
    if arm_resp.success:
        rospy.loginfo("Vehicle armed.")
    else:
        rospy.logerr("Arming failed.")

    rospy.wait_for_service("iris_0/mavros/set_mode")
    set_mode = rospy.ServiceProxy("iris_0/mavros/set_mode", SetMode)
    mode_resp = set_mode(custom_mode="OFFBOARD")
    if mode_resp.mode_sent:
        rospy.loginfo("OFFBOARD mode enabled.")
    else:
        rospy.logwarn("Failed to enter OFFBOARD mode.")

    # === 主循环 ===
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()
        twist.header.stamp = rospy.Time.now()
        vel_pub.publish(twist)

        # 丢失目标处理
        if find_cnt - find_cnt_last == 0:
            if not get_time:
                not_find_time = rospy.get_time()
                get_time = True
            if rospy.get_time() - not_find_time > 2.0:
                twist.twist.linear.x = 0.0
                twist.twist.linear.y = 0.0
                twist.twist.linear.z = 0.0
                rospy.logwarn("Target lost: HOVER")
                vel_pub.publish(twist)
                get_time = False

        find_cnt_last = find_cnt
