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
    //ROS_INFO("x: [%f]", msg->pose.position.x);
    //ROS_INFO("y: [%f]", msg->pose.position.y);
    //ROS_INFO("z: [%f]", msg->pose.position.z);
    current_position = *msg;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel");         // node name
    ros::NodeHandle nh;                  // node handle
 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>  //mavros_msgs::State：属于 mavros_msgs 包中的 State 消息类型，用于表示 MAVROS 无人机的状态信息。
            ("iris_1/mavros/state", 10, state_cb);                       //订阅 mavros/state 话题，用于获取无人机的状态信息。（如飞行模式、是否解锁等）。 在 PX4 + MAVROS 系统中，/mavros/state 话题用于发布无人机的状态，包括： 当前飞行模式（如 OFFBOARD、AUTO.LOITER）;是否解锁（马达是否启动）等
            
    ros::Subscriber get_point = nh.subscribe<geometry_msgs::PoseStamped>   //订阅 mavros/local_position/pose 话题，用于获取无人机的位置信息。PoseStamped 是 ROS 中的一个标准消息类型，表示一个带有时间戳的三维位姿（位置 + 姿态）。
            ("iris_1/mavros/local_position/pose", 10, getpointfdb);                //"mavros/local_position/pose"，MAVROS 提供的一个 本地位置 话题。
            
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("iris_1/mavros/setpoint_position/local", 10);                         //"mavros/setpoint_position/local"，这是 MAVROS 话题，用于接收无人机的本地目标位置指令。当你想让无人机飞到一个指定的目标位置时，可以向该话题发布 PoseStamped 消息。

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/iris_1/mavros/setpoint_velocity/cmd_vel", 10);      
    /*这行代码是 ROS 服务客户端 的一个定义，用于创建一个客户端，用于通过 MAVROS 控制 PX4 无人机的 解锁 (arming) 和 上锁 (disarming) 操作。 
    <mavros_msgs::CommandBool>这是 MAVROS 定义的一个标准消息类型，用于解锁/上锁无人机的服务请求。
    mavros/cmd/arming 是 MAVROS 提供的用于 解锁/上锁 无人机的 ROS 服务名称。*/
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("iris_1/mavros/cmd/arming");//向 PX4 发送解锁（arm）或上锁（disarm）请求
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("iris_1/mavros/set_mode");//	向 PX4 请求切换飞行模式（如 OFFBOARD、AUTO.TAKEOFF 等）

    //the setpoint publishing rate MUST be faster than 2Hz  2Hz表示每秒执行两次
    ros::Rate rate(20.0f);


    // wait for FCU connection    FCU：飞行控制单元   连接指的就是 PX4 系统是否已经启动了，并成功连接到了 ROS 系统。
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();                        //让 ROS 处理一次 订阅器 (Subscriber) 和 回调函数。
        rate.sleep();
    }
 

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 3;
    
    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 1;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 0;
    
 
    //send a few setpoints before starting
    //在 PX4 的 OFFBOARD 模式 中，飞控需要在切换模式前持续接收到目标点消息，否则无法进入 OFFBOARD 模式。
    //PX4 需要确认外部计算机能够持续提供控制指令，否则会自动回退到 POSCTL（位置控制）模式。
    //提前发送 100 次目标点，确保 PX4 在 OFFBOARD 模式切换时已经收到足够的数据。
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        // ROS_INFO("%d",i);
        ros::spinOnce();
        rate.sleep();
    }
 
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;  
    arm_cmd.request.value = true;       //解锁 (Arm)：允许电机旋转，进入可飞行状态
 
    ros::Time last_request = ros::Time::now();
    bool takeoff_done = false;
    while(ros::ok()){
        ROS_INFO("%s",current_state.mode.c_str());
        ROS_INFO("%f",(ros::Time::now() - last_request).toSec());
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0f))){
            if( set_mode_client.call(offb_set_mode) &&      //set_mode_client.call(offb_set_mode)向指定的服务端发送一个请求，并等待服务端返回响应结果。
                offb_set_mode.response.mode_sent){               //offb_set_mode.response.mode_sent 是 ROS 服务响应 中的一个布尔值 (Boolean)，用于指示 飞行模式切换命令 是否已成功发送到 PX4 飞控。
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0f))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){     //检查解锁是否成功。
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        
        if (!takeoff_done) {
            // 起飞阶段：发布目标位置
            local_pos_pub.publish(pose);
            if (fabs(current_position.pose.position.z - 3.0) < 0.1) {
                takeoff_done = true;
                ROS_INFO("Takeoff finished. Start velocity control.");
            }
        }else {
            // 起飞完成后，发布速度指令
            local_pos_pub.publish(pose);
            vel_pub.publish(twist);
        }
        
        // local_pos_pub.publish(pose);
        // vel_pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}