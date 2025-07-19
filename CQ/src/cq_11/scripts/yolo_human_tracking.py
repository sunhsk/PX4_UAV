import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import sys 
from pyquaternion import Quaternion
from darknet_ros_msgs.msg import BoundingBoxes
import math

def darknet_callback(data):
    global find_cnt, twist, cmd, target_height_mask, target_height,theta, get_time
    for target in data.bounding_boxes:   #YOLO 检测的所有目标框组成的列表。
        if(target.id==0):                
            print('find human')
            z = height / math.sin(theta)           #无人机到人的直线距离
            #(u, v) 是目标框中心像素。
            u = (target.xmax+target.xmin)/2
            v = (target.ymax+target.ymin)/2

            #目标相对图像中心的像素偏移
            u_ = u-u_center                     
            v_ = v-v_center

            #加负号是因为采用负反馈控制：如果目标在图像右边，应该向左移动，即误差为正时速度为负。
            u_velocity = -Kp_xy*u_
            v_velocity = -Kp_xy*v_
            
            #从图像坐标转换为世界坐标系速度：
            x_velocity = v_velocity*z/(v_*math.cos(theta)+fy*math.sin(theta))
            y_velocity = (z*u_velocity-u_*math.cos(theta)*x_velocity)/fx

            twist.linear.x = x_velocity
            twist.linear.y = y_velocity
            twist.linear.z = Kp_z*(target_height-height)
            cmd = ''
            find_cnt = find_cnt + 1
            get_time = False
    
        
def local_pose_callback(data):
    global height, target_height, target_set
    height = data.pose.position.z    
    if not target_set:
        target_height = height     
        target_set = True    

def cam_pose_callback(data):
    global theta
    #四元数（orientation）转换为z = height / math.sin(theta)一个 Quaternion 类的对象，用于姿态解算。它提供了四元数→欧拉角的便捷转换函数。
    q = Quaternion(data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z)
    #q.yaw_pitch_roll 会返回一个包含 (yaw, pitch, roll) 的三元组（单位是弧度）[1] 表示取出中间的 俯仰角 pitch（θ），即相机上下抬头的角度。
    theta = q.yaw_pitch_roll[1]
            
if __name__ == "__main__":
    ''''height: 当前无人机高度；

    target_height: 目标高度（初始设定为首次检测到目标的高度）；

    target_set: 标志位，判断是否已经设置过 target_height；

    find_cnt: 当前帧检测到目标的次数（用于判断是否丢失）；

    find_cnt_last: 上一帧检测到目标的次数；

    not_find_time: 记录“开始丢目标”的时间；

    get_time: 是否已经记录了“丢失开始时间”；

    twist: 控制无人机的线速度消息；

    cmd: 控制无人机飞行模式的指令（如HOVER）；

    theta: 相机俯仰角（pitch）；'''
    height = 0                           
    target_height = 0
    target_set = False
    find_cnt = 0
    find_cnt_last = 0
    not_find_time = 0
    get_time = False
    twist = Twist()
    cmd = String()
    theta = 0
    u_center=640/2                     # u_center, v_center: 图像的中心点坐标（假设图像分辨率为 640x360）；
    v_center=360/2
    fx = 205.46963709898583            # 相机的内参（焦距）；
    fy = 205.46963709898583
    Kp_xy = 0.5                        # Kp_xy, Kp_z: 比例控制器的增益，用于 XY 方向和 Z 方向的速度控制。
    Kp_z = 1
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    rospy.init_node('yolo_human_tracking')
    #订阅 YOLO 识别输出的边框消息； BoundingBoxes:data type
    rospy.Subscriber("/uav_"+vehicle_id+"/darknet_ros/bounding_boxes", BoundingBoxes, darknet_callback,queue_size=1) 
    #订阅本机的实际飞行高度；
    rospy.Subscriber(vehicle_type+'_'+vehicle_id+"/mavros/local_position/pose", PoseStamped, local_pose_callback,queue_size=1)
    #订阅相机姿态信息（四元数）；用于计算俯仰角 theta
    rospy.Subscriber('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cam_pose', PoseStamped, cam_pose_callback,queue_size=1)
    #发布无人机的线速度命令（机体坐标系）。 
    cmd_vel_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd_vel_flu', Twist, queue_size=1)
    #发布控制指令，如 HOVER 等。
    cmd_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd', String, queue_size=1)
    # 控制主循环（60Hz）  表示每秒循环 60 次（执行频率 60Hz）
    rate = rospy.Rate(60) 
    while not rospy.is_shutdown():
        rate.sleep()
        #每帧发布当前的速度命令和控制指令。
        cmd_vel_pub.publish(twist)
        cmd_pub.publish(cmd)
        #判断是否丢失目标并执行“悬停”  如果连续两帧都没有检测到目标，说明目标丢失；
        if find_cnt - find_cnt_last == 0:
            if not get_time:
                not_find_time = rospy.get_time()    #开始计时，记录丢失时间。
                get_time = True
            if rospy.get_time() - not_find_time > 2.0:   #如果目标连续 2秒 没有出现，则发送悬停命令，并清空速度。
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                cmd = 'HOVER'
                print(cmd)
                get_time = False
        find_cnt_last = find_cnt                             #记录当前帧检测目标次数，用于下一帧比对。