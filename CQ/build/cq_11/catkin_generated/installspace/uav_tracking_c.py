#!/usr/bin/env python3
# coding=utf-8

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
import math
import sys
from mavros_msgs.srv import SetMode, CommandBool




# 控制参数（可调）
'''Kp_xy = 0.7
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

def run_position_tracking(our_uav,our_uav_id,enemy_uav,enemy_uav_id):
    # === 全局变量 ===
    global self_pose,target_pose,twist,cmd,find_cnt,find_cnt_last,not_find_time,get_time
    self_pose = PoseStamped()
    target_pose = PoseStamped()
    twist = TwistStamped()
    cmd = String()
    find_cnt = 0
    find_cnt_last = 0
    not_find_time = 0
    get_time = False
# 获取启动参数（如无人机类型、编号）
    #vehicle_type = sys.argv[1]  # 例如 "uav"
    #vehicle_id = sys.argv[2]    # 例如 "0"

    # rospy.init_node('position_tracking_node')

    # === 订阅话题 ===
    # 本机位置
    rospy.Subscriber(our_uav + '_' + str(our_uav_id) + '/mavros/vision_pose/pose',
                     PoseStamped, self_pose_callback, queue_size=1)
    
    # 目标无人机位置（假设为非法无人机，id 固定为 "1"）
    rospy.Subscriber(enemy_uav + '_' + str(enemy_uav_id)  +'/mavros/vision_pose/pose',
                     PoseStamped, target_pose_callback, queue_size=1)

    # === 发布器 ===
    cmd_vel_pub = rospy.Publisher(our_uav + '_' + str(our_uav_id) + '/mavros/setpoint_velocity/cmd_vel',
                                  TwistStamped, queue_size=1)
    
     # === 控制队形 zanshi meiyong ===
    leader_cmd_sub = rospy.Publisher('xtdrone/leader/cmd',
                                  String, queue_size=1)
    
    rospy.loginfo("Pre-flight OFFBOARD setpoint streaming...")
    for _ in range(100):  # 发布100次
        twist.header.stamp = rospy.Time.now()
        twist.twist.linear.z = 0.1 
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.015)

    # 切换 OFFBOARD 模式
    cmd_pub = rospy.ServiceProxy(our_uav + '_' + str(our_uav_id) + '/mavros/set_mode',SetMode)
    rospy.wait_for_service(our_uav + '_' + str(our_uav_id) + '/mavros/set_mode')
    cmd_pub(custom_mode="OFFBOARD")

    #解锁飞行器
    arming_srv = rospy.ServiceProxy(our_uav + '_' + str(our_uav_id) + '/mavros/cmd/arming', CommandBool)
    rospy.wait_for_service(our_uav + '_' + str(our_uav_id) + '/mavros/cmd/arming')
    
    # # 切换到 OFFBOARD 模式
    # try:
    #     resp = cmd_pub(custom_mode="OFFBOARD")
    #     if resp.mode_sent:
    #         rospy.loginfo("OFFBOARD mode enabled.")
    #     else:
    #         rospy.logwarn("Failed to switch to OFFBOARD mode.")
    # except rospy.ServiceException as e:
    #     rospy.logerr("SetMode service call failed: %s", e)

    # # 解锁无人机
    # arm_resp = arming_srv(True)
    # if arm_resp.success:
    #     rospy.loginfo("Vehicle armed")
    # else:
    #     rospy.logerr("Arming failed")


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

        find_cnt_last = find_cnt'''

#!/usr/bin/env python
# coding=utf-8


class UAVPositionTracker:
    def __init__(self, our_uav, our_uav_id, enemy_uav, enemy_uav_id, height_offset):
        self.our_uav = our_uav
        self.our_uav_id = our_uav_id
        self.enemy_uav = enemy_uav
        self.enemy_uav_id = enemy_uav_id
        self.height_offset = height_offset

        # 状态变量
        self.self_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.twist = TwistStamped()
        self.cmd = String()
        self.find_cnt = 0
        self.find_cnt_last = 0
        self.not_find_time = 0
        self.get_time = False

        # 控制参数
        self.Kp_xy = 0.7
        self.Kp_z = 0.6

        # ROS通信对象
        self.cmd_vel_pub = None
        self.leader_cmd_pub = None
        self.cmd_pub = None
        self.arming_srv = None

    def self_pose_callback(self, data):
        self.self_pose = data

    def target_pose_callback(self, data):
        self.target_pose = data
        sx = self.self_pose.pose.position.x
        sy = self.self_pose.pose.position.y
        sz = self.self_pose.pose.position.z

        tx = self.target_pose.pose.position.x
        ty = self.target_pose.pose.position.y
        tz = self.target_pose.pose.position.z
        
        desired_z = tz + self.height_offset

        dx = tx - sx
        dy = ty - sy
        dz = desired_z - sz

        self.twist.twist.linear.x = self.Kp_xy * dx
        self.twist.twist.linear.y = self.Kp_xy * dy
        self.twist.twist.linear.z = self.Kp_z  * dz

        self.cmd.data = ''
        self.find_cnt += 1
        self.get_time = False

    def run(self):
        # === ROS通信对象 ===
        self.cmd_vel_pub = rospy.Publisher(
            self.our_uav + '_' + str(self.our_uav_id) + '/mavros/setpoint_velocity/cmd_vel',
            TwistStamped, queue_size=1)
        self.leader_cmd_pub = rospy.Publisher('xtdrone/leader/cmd', String, queue_size=1)
        
        # 订阅自身与目标无人机的位置
        rospy.Subscriber(self.our_uav + '_' + str(self.our_uav_id) + '/mavros/vision_pose/pose',
                         PoseStamped, self.self_pose_callback, queue_size=1)
        rospy.Subscriber(self.enemy_uav + '_' + str(self.enemy_uav_id)  +'/mavros/vision_pose/pose',
                         PoseStamped, self.target_pose_callback, queue_size=1)

        # === 服务端口 ===
        self.cmd_pub = rospy.ServiceProxy(
            self.our_uav + '_' + str(self.our_uav_id) + '/mavros/set_mode', SetMode)
        rospy.wait_for_service(self.our_uav + '_' + str(self.our_uav_id) + '/mavros/set_mode')
        self.arming_srv = rospy.ServiceProxy(
            self.our_uav + '_' + str(self.our_uav_id) + '/mavros/cmd/arming', CommandBool)
        rospy.wait_for_service(self.our_uav + '_' + str(self.our_uav_id) + '/mavros/cmd/arming')

        rospy.loginfo("Pre-flight OFFBOARD setpoint streaming...")
        for _ in range(100):
            self.twist.header.stamp = rospy.Time.now()
            self.twist.twist.linear.z = 0.1
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(0.015)

        # 切换OFFBOARD模式
        self.cmd_pub(custom_mode="OFFBOARD")
        # self.arming_srv(True)  # 可解锁飞行器

        rate = rospy.Rate(60)
        msg = String()
        msg.data = "T"

        while not rospy.is_shutdown():
            self.leader_cmd_pub.publish(msg)
            rate.sleep()
            self.cmd_vel_pub.publish(self.twist)

            # 丢失目标则悬停
            if self.find_cnt - self.find_cnt_last == 0:
                if not self.get_time:
                    self.not_find_time = rospy.get_time()
                    self.get_time = True
                if rospy.get_time() - self.not_find_time > 2.0:
                    self.twist.twist.linear.x = 0.0
                    self.twist.twist.linear.y = 0.0
                    self.twist.twist.linear.z = 0.0
                    self.cmd_pub(custom_mode='HOVER')
                    print("Target lost: HOVER")
                    self.get_time = False

            self.find_cnt_last = self.find_cnt

'''# 示例多线程调用
if __name__ == "__main__":
    rospy.init_node("multi_uav_tracking_demo")
    import threading

    uav_ids = [2, 3, 4]
    trackers = []
    threads = []
    for uav_id in uav_ids:
        tracker = UAVPositionTracker('iris', uav_id, 'iris', 11)
        trackers.append(tracker)
        t = threading.Thread(target=tracker.run)
        t.start()
        threads.append(t)
    for t in threads:
        t.join()'''
