import rospy
import threading
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
import sys
from take_off import multi_uav_offboard_hover

# 如果你想测试，可以如下调用：
if __name__ == "__main__":
    # 例如起飞iris类型的0号和2号无人机
    stop_events, threads = multi_uav_offboard_hover("iris", [2, 3,4])
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")

    for evt in stop_events:
        evt.set()
    for t in threads:
        t.join()
