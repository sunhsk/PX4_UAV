#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_velocity_controller");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("standard_vtol_0/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    ros::Rate rate(20.0);  // 20Hz

    double radius = 8.0;   // 半径
    double speed = 3.0;    // 线速度 (m/s)
    double omega = speed / radius; // 角速度

    ros::Time start_time = ros::Time::now();

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        double t = (current_time - start_time).toSec();

        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = -speed * sin(omega * t);
        vel_msg.linear.y = speed * cos(omega * t);
        vel_msg.linear.z = 0.0;
        
        // 计算当前位置的朝向角，使无人机保持正方向飞行
        double theta = atan2(vel_msg.linear.y, vel_msg.linear.x);
        // 计算朝向角速度，使用PD控制器简化实现
        double target_yaw = theta;
        static double last_yaw_error = 0;
        double current_yaw = 0; // 实际应用中应通过IMU获取当前偏航角
        double yaw_error = target_yaw - current_yaw;
        // 角度误差归一化到[-pi, pi]
        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
        
        // PD控制参数，实际应用中需要调整
        double Kp = 1.0;
        double Kd = 0.1;
        double yaw_rate = Kp * yaw_error + Kd * (yaw_error - last_yaw_error) * rate.expectedCycleTime().toSec();
        last_yaw_error = yaw_error;
        
        vel_msg.angular.z = yaw_rate;

        vel_pub.publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
    