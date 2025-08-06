import rospy
import math
import threading
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
import sys
sys.path.insert(0, "/home/hsk/CQ/src/cq_11/scripts/main")
from take_off import multi_uav_offboard_hover
sys.path.insert(0, "/home/hsk/CQ/src/cq_11/scripts/main/formation_b")
import formation_b.leader as leader 
import formation_b.follower as follower
import formation_b.avoid as avoid 
import formation_b.formation_dict as formation_dict
sys.path.insert(0, "/home/hsk/CQ/src/cq_11/scripts/main/tacking")
# import tacking.uav_tracking_c
from tacking.uav_tracking_c import UAVPositionTracker

self_pose_a = PoseStamped() 
self_pose_b = PoseStamped() 
self_pose_c = PoseStamped() 
def self_pose_callback_a(data):
    global self_pose_a
    self_pose_a = data
def self_pose_callback_b(data):
    global self_pose_b
    self_pose_b = data
def self_pose_callback_c(data):
    global self_pose_c
    self_pose_c = data


# 如果你想测试，可以如下调用：
if __name__ == "__main__":
    # 例如起飞iris类型的0号和2号无人机
    uav_ids = [2, 3, 4]
    stop_events, threads = multi_uav_offboard_hover("iris", uav_ids)
    rospy.Subscriber('iris_' + str(2) + '/mavros/vision_pose/pose',
                     PoseStamped, self_pose_callback_a, queue_size=1)
    rospy.Subscriber('iris_' + str(3) + '/mavros/vision_pose/pose',
                     PoseStamped, self_pose_callback_b, queue_size=1)
    rospy.Subscriber('iris_' + str(4) + '/mavros/vision_pose/pose',
                     PoseStamped, self_pose_callback_c, queue_size=1)
    x_num = self_pose_a.pose.position.x + self_pose_b.pose.position.x + self_pose_c.pose.position.x  
    y_num = self_pose_a.pose.position.y + self_pose_b.pose.position.y + self_pose_c.pose.position.y    
    z_num = self_pose_a.pose.position.z + self_pose_b.pose.position.z + self_pose_c.pose.position.z               
    trackers = []
    threads = []
    for uav_id in uav_ids:
        tracker = UAVPositionTracker('iris', uav_id, 'iris', 11,uav_id)
        trackers.append(tracker)
        t = threading.Thread(target=tracker.run)
        t.start()
        threads.append(t)
    
    print(11)

    try:
        rospy.spin()                    #保持ROS节点运行，等待用户中断或退出
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")   #捕捉到 Ctrl+C 退出时打印日志

    # leader.run_leader()


       # 当程序结束时，关闭所有无人机控制线程
    for evt in stop_events: 
        evt.set()          # 通知线程停止循环
    for t in threads:
        t.join()        # t.join() 会 阻塞当前主线程，等待线程 t 执行完毕后才继续往下执行。等待所有线程结束”，确保所有后台线程都安全、完整地退出后，程序才继续执行或退出。
    
