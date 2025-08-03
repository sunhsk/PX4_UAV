import rospy
import threading
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

TAKEOFF_Z = 10.0
SETPOINT_HZ = 20

def publish_setpoint_loop(uav_type, uav_id, takeoff_z, stop_event):
    topic = f"{uav_type}_{uav_id}/mavros/setpoint_position/local"
    pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
    rate = rospy.Rate(SETPOINT_HZ)
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = takeoff_z

    rospy.loginfo(f"UAV {uav_id}: Start publishing setpoints...")
    while not rospy.is_shutdown() and not stop_event.is_set():
        pose.header.stamp = rospy.Time.now()
        pub.publish(pose)
        rate.sleep()

def arm_and_set_mode(uav_type, uav_id):
    ns = f"{uav_type}_{uav_id}"
    rospy.wait_for_service(f"{ns}/mavros/cmd/arming")
    rospy.wait_for_service(f"{ns}/mavros/set_mode")
    try:
        arming_client = rospy.ServiceProxy(f"{ns}/mavros/cmd/arming", CommandBool)
        mode_client = rospy.ServiceProxy(f"{ns}/mavros/set_mode", SetMode)

        mode_resp = mode_client(0, "OFFBOARD")
        if mode_resp.mode_sent:
            rospy.loginfo(f"UAV {uav_id}: Mode set to OFFBOARD")
        else:
            rospy.logwarn(f"UAV {uav_id}: Failed to set mode to OFFBOARD")

        arm_resp = arming_client(True)
        if arm_resp.success:
            rospy.loginfo(f"UAV {uav_id}: Armed successfully")
        else:
            rospy.logwarn(f"UAV {uav_id}: Failed to arm")

    except rospy.ServiceException as e:
        rospy.logerr(f"UAV {uav_id}: Service call failed: {e}")

def multi_uav_offboard_hover(uav_type, uav_ids, takeoff_z=TAKEOFF_Z):
    """
    uav_type: str, 无人机类型名称，例如 "iris"
    uav_ids: list of int, 指定起飞的无人机编号列表，例如 [0, 2, 5]
    takeoff_z: float, 起飞高度，默认10米
    """
    if not rospy.core.is_initialized():
        rospy.init_node("multi_uav_offboard_hover", anonymous=True)

    stop_events = []
    threads = []

    for uav_id in uav_ids:
        evt = threading.Event()
        stop_events.append(evt)
        t = threading.Thread(target=publish_setpoint_loop, args=(uav_type, uav_id, takeoff_z, evt))
        t.start()
        threads.append(t)

    rospy.sleep(5)  # 等待setpoint发布稳定

    for uav_id in uav_ids:
        arm_and_set_mode(uav_type, uav_id)

    rospy.loginfo(f"UAVs {uav_ids} are in OFFBOARD and hovering at {takeoff_z}m.")

    return stop_events, threads

