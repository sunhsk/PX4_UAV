#!/usr/bin/env python3
# coding=utf-8

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
import math
import sys
from mavros_msgs.srv import SetMode, CommandBool


# === 全局变量 ===
self_pose = PoseStamped()
target_pose = PoseStamped()
twist = TwistStamped()
cmd = String()
find_cnt = 0
find_cnt_last = 0
not_find_time = 0
get_time = False

# 控制参数（可调）
Kp_xy = 0.7
Kp_z = 0.6

def self_pose_callback(data):
    global self_pose
    self_pose = data

def target_pose_callback(data):
    global target_pose, twist, cmd, find_cnt, get_time
    target_pose = data

    # 提取自身和目标的位置
    sx = self_pose.pose.position.x
    sy = self_pose.pose.position.y
    sz = self_pose.pose.position.z

    tx = target_pose.pose.position.x
    ty = target_pose.pose.position.y
    tz = target_pose.pose.position.z
    
    #设置高度偏移
    height_offset = 2.0  # 你想保持在目标上方2米
    desired_z = tz + height_offset

    # 计算误差（单位：米）
    dx = tx - sx
    dy = ty - sy
    # dz = tz - sz
    dz = desired_z - sz

    # 比例控制器输出速度
    twist.twist.linear.x = Kp_xy * dx
    twist.twist.linear.y = Kp_xy * dy
    twist.twist.linear.z = Kp_z  * dz

    cmd.data = ''
    find_cnt += 1
    get_time = False

if __name__ == "__main__":
    # 获取启动参数（如无人机类型、编号）
    #vehicle_type = sys.argv[1]  # 例如 "uav"
    #vehicle_id = sys.argv[2]    # 例如 "0"

    rospy.init_node('position_tracking_node')

    # === 订阅话题 ===
    # 本机位置
    rospy.Subscriber('iris_0/mavros/vision_pose/pose',
                     PoseStamped, self_pose_callback, queue_size=1)
    
    # 目标无人机位置（假设为非法无人机，id 固定为 "1"）
    rospy.Subscriber('iris_1/mavros/vision_pose/pose',
                     PoseStamped, target_pose_callback, queue_size=1)

    # === 发布器 ===
    cmd_vel_pub = rospy.Publisher('iris_0/mavros/setpoint_velocity/cmd_vel',
                                  TwistStamped, queue_size=1)
    
     # === 控制队形 ===
    leader_cmd_sub = rospy.Publisher('xtdrone/leader/cmd',
                                  String, queue_size=1)
    
    rospy.loginfo("Pre-flight OFFBOARD setpoint streaming...")
    for _ in range(100):  # 发布100次
        twist.header.stamp = rospy.Time.now()
        twist.twist.linear.z = 0.1 
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.015)

    # 切换 OFFBOARD 模式
    cmd_pub = rospy.ServiceProxy('iris_0/mavros/set_mode',SetMode)
    rospy.wait_for_service('iris_0/mavros/set_mode')
    cmd_pub(custom_mode="OFFBOARD")

    #解锁飞行器
    arming_srv = rospy.ServiceProxy('iris_0/mavros/cmd/arming', CommandBool)
    rospy.wait_for_service('iris_0/mavros/cmd/arming')
    
    # 切换到 OFFBOARD 模式
    try:
        resp = cmd_pub(custom_mode="OFFBOARD")
        if resp.mode_sent:
            rospy.loginfo("OFFBOARD mode enabled.")
        else:
            rospy.logwarn("Failed to switch to OFFBOARD mode.")
    except rospy.ServiceException as e:
        rospy.logerr("SetMode service call failed: %s", e)

    # 解锁无人机
    arm_resp = arming_srv(True)
    if arm_resp.success:
        rospy.loginfo("Vehicle armed")
    else:
        rospy.logerr("Arming failed")


    # === 控制主循环 ===
    rate = rospy.Rate(60)  # 60Hz 发布频率

    ##--------------#
    msg = String()
    msg.data = "T"

    while not rospy.is_shutdown():
        
        leader_cmd_sub.publish(msg)

        rate.sleep()
        cmd_vel_pub.publish(twist)

        # 如果目标丢失超时，则悬停
        if find_cnt - find_cnt_last == 0:
            if not get_time:
                not_find_time = rospy.get_time()
                get_time = True
            if rospy.get_time() - not_find_time > 2.0:
                twist.twist.linear.x = 0.0
                twist.twist.linear.y = 0.0
                twist.twist.linear.z = 0.0
                #cmd.data = 'HOVER'
                cmd_pub(custom_mode='HOVER')
                print("Target lost: HOVER")
                get_time = False

        find_cnt_last = find_cnt
