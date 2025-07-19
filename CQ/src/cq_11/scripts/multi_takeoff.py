#!/usr/bin/env python
import rospy
import threading
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

UAV_NUM = 6
TAKEOFF_Z = 2.0
SETPOINT_HZ = 20

def publish_setpoint_loop(uav_id, stop_event):
    topic = "/iris_%d/mavros/setpoint_position/local" % uav_id
    pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
    rate = rospy.Rate(SETPOINT_HZ)
    pose = PoseStamped()
    pose.header.frame_id = "map"  # 或 "world"，视你的 TF 而定
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = TAKEOFF_Z

    rospy.loginfo("UAV %d: Start publishing setpoints..." % uav_id)
    while not rospy.is_shutdown() and not stop_event.is_set():
        pose.header.stamp = rospy.Time.now()
        pub.publish(pose)
        rate.sleep()

def arm_and_set_mode(uav_id):
    ns = "/iris_" + str(uav_id)
    rospy.wait_for_service(ns + "/mavros/cmd/arming")
    rospy.wait_for_service(ns + "/mavros/set_mode")
    try:
        arming_client = rospy.ServiceProxy(ns + "/mavros/cmd/arming", CommandBool)
        mode_client = rospy.ServiceProxy(ns + "/mavros/set_mode", SetMode)
        
        # 切换到 OFFBOARD 模式
        mode_resp = mode_client(0, "OFFBOARD")
        if mode_resp.mode_sent:
            rospy.loginfo("UAV %d: Mode set to OFFBOARD" % uav_id)
        else:
            rospy.logwarn("UAV %d: Failed to set mode to OFFBOARD" % uav_id)

        # 解锁
        arm_resp = arming_client(True)
        if arm_resp.success:
            rospy.loginfo("UAV %d: Armed successfully" % uav_id)
        else:
            rospy.logwarn("UAV %d: Failed to arm" % uav_id)

    except rospy.ServiceException as e:
        rospy.logerr("UAV %d: Service call failed: %s" % (uav_id, e))

if __name__ == '__main__':
    rospy.init_node("multi_uav_offboard_hover")

    # 启动每架无人机的 setpoint 发布线程
    stop_events = []
    threads = []
    for i in range(UAV_NUM):
        evt = threading.Event()
        stop_events.append(evt)
        t = threading.Thread(target=publish_setpoint_loop, args=(i, evt))
        t.start()
        threads.append(t)

    # 等待 setpoint 足够时间再设置模式（>2s）
    rospy.sleep(5)

    for i in range(UAV_NUM):
        arm_and_set_mode(i)

    rospy.loginfo("All UAVs are in OFFBOARD and hovering at 2m. Press Ctrl+C to stop.")

    try:
        rospy.spin()  # 保持主线程运行
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")

    for evt in stop_events:
        evt.set()
    for t in threads:
        t.join()
