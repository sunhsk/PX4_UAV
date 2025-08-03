#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
 
 
mavros_msgs::State current_state;     
geometry_msgs::PoseStamped current_position;
 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void getpointfdb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_leader");         // node name
    ros::NodeHandle nh;                     // node handle
 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>  //mavros_msgs::State：属于 mavros_msgs 包中的 State 消息类型，用于表示 MAVROS 无人机的状态信息。
            ("iris_0/mavros/state", 10, state_cb);                       //订阅 mavros/state 话题，用于获取无人机的状态信息。（如飞行模式、是否解锁等）。 在 PX4 + MAVROS 系统中，/mavros/state 话题用于发布无人机的状态，包括： 当前飞行模式（如 OFFBOARD、AUTO.LOITER）;是否解锁（马达是否启动）等
            
    ros::Subscriber get_point = nh.subscribe<geometry_msgs::PoseStamped>   //订阅 mavros/local_position/pose 话题，用于获取无人机的位置信息。PoseStamped 是 ROS 中的一个标准消息类型，表示一个带有时间戳的三维位姿（位置 + 姿态）。
            ("iris_0/mavros/local_position/pose", 10, getpointfdb);                //"mavros/local_position/pose"，MAVROS 提供的一个 本地位置 话题。
            
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("iris_0/mavros/setpoint_position/local", 10);                         //"mavros/setpoint_position/local"，这是 MAVROS 话题，用于接收无人机的本地目标位置指令。当你想让无人机飞到一个指定的目标位置时，可以向该话题发布 PoseStamped 消息。

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/iris_0/mavros/setpoint_velocity/cmd_vel", 10); 

    ros::Rate rate(20.0f);



    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 2;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 0;
    
 
    while(ros::ok()){
        
        vel_pub.publish(twist);
        
        // local_pos_pub.publish(pose);
        // vel_pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}