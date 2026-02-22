#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <algorithm> // 用于 std::clamp

unsigned short velocity_mask = 1991;

mavros_msgs::State current_state;
nav_msgs::Odometry current_odom;
geometry_msgs::Twist current_cmd_vel;
bool odom_received = false;
bool cmd_vel_received = false;
bool takeoff_done = false;
ros::Time last_cmd_vel_time;

// ================= 全局参数 (默认值) =================
// 这些值会被 launch 文件覆盖
float target_height = 1.0;  // 目标高度
float kp_z = 1.0;           // 垂直P参数
float max_z_vel = 0.6;      // 最大垂直速度 (起飞速度)
float max_xy_vel = 1.0;     // 【新增】最大水平速度限制 (m/s)
float max_yaw_rate = 1.0;   // 【新增】最大旋转速度限制 (rad/s)

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_odom = *msg;
    odom_received = true;
}

void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg){
    current_cmd_vel = *msg;
    last_cmd_vel_time = ros::Time::now();
    cmd_vel_received = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_cmd_vel_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    // ================= 1. 读取 Launch 文件参数 =================
    nh_private.param<float>("target_height", target_height, 1.0);
    nh_private.param<float>("kp_z", kp_z, 1.5);
    nh_private.param<float>("max_z_vel", max_z_vel, 0.6);      // 垂直限速
    nh_private.param<float>("max_xy_vel", max_xy_vel, 1.0);    // 水平限速
    nh_private.param<float>("max_yaw_rate", max_yaw_rate, 1.0);// 旋转限速

    ROS_INFO("Bridge Params Loaded:");
    ROS_INFO(" - Target Height: %.2f m", target_height);
    ROS_INFO(" - Max Z Vel: %.2f m/s", max_z_vel);
    ROS_INFO(" - Max XY Vel: %.2f m/s", max_xy_vel);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, odom_cb);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, cmd_vel_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    ros::Rate rate(30.0); 

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO_THROTTLE(2, "Waiting for FCU connection...");
    }

    ROS_INFO("Bridge Connected. Waiting for Odom...");
    while(ros::ok() && !odom_received){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Odom received. Ready.");

    mavros_msgs::PositionTarget pos_target;
    
    while(ros::ok())
    {
        tf2::Quaternion quat;
        tf2::fromMsg(current_odom.pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        pos_target.header.stamp = ros::Time::now();
        pos_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        pos_target.type_mask = velocity_mask;

        // --- 垂直控制 ---
        float current_z = current_odom.pose.pose.position.z;
        float err_z = target_height - current_z;
        float target_vz = err_z * kp_z;

        // 垂直限幅 (使用参数)
        if(target_vz > max_z_vel) target_vz = max_z_vel;
        if(target_vz < -max_z_vel) target_vz = -max_z_vel;

        // --- 起飞判定 ---
        if (!takeoff_done) {
            if (std::abs(err_z) < 0.15) {
                takeoff_done = true;
                ROS_WARN("=== Takeoff Complete! Switching to Tracking Mode ===");
            } else {
                ROS_INFO_THROTTLE(1.0, "Taking off... Current Z: %.2f / %.2f", current_z, target_height);
            }
        }

        // --- 水平控制 ---
        float target_vx_local = 0;
        float target_vy_local = 0;
        float target_yaw_rate = 0;

        if(takeoff_done && cmd_vel_received && (ros::Time::now() - last_cmd_vel_time).toSec() < 0.5) {
            float cmd_x = current_cmd_vel.linear.x; 
            float cmd_y = current_cmd_vel.linear.y; 

            // 旋转到 ENU 坐标系
            target_vx_local = cmd_x * cos(yaw) - cmd_y * sin(yaw);
            target_vy_local = cmd_x * sin(yaw) + cmd_y * cos(yaw);
            
            
            target_yaw_rate = current_cmd_vel.angular.z;

            // ================= 2. 增加安全限幅逻辑 =================
            
            // A. 水平速度限幅 (按比例缩放，保持方向不变)
            float speed_xy = std::hypot(target_vx_local, target_vy_local); // 计算合速度
            if (speed_xy > max_xy_vel) {
                float ratio = max_xy_vel / speed_xy;
                target_vx_local *= ratio;
                target_vy_local *= ratio;
            }

            // B. 旋转速度限幅
            if (target_yaw_rate > max_yaw_rate) target_yaw_rate = max_yaw_rate;
            if (target_yaw_rate < -max_yaw_rate) target_yaw_rate = -max_yaw_rate;
        }

        pos_target.velocity.x = target_vx_local;
        pos_target.velocity.y = target_vy_local;
        pos_target.velocity.z = target_vz;
        pos_target.yaw_rate = target_yaw_rate;

        // 打印信息
        if(takeoff_done && (std::abs(target_vx_local) > 0.05 || std::abs(target_vy_local) > 0.05 || std::abs(target_yaw_rate) > 0.05)) {
            ROS_INFO_THROTTLE(0.5, "TRACKING: Vx:%.2f Vy:%.2f Yaw:%.2f | H:%.2f", 
            target_vx_local, target_vy_local, target_yaw_rate, current_z);
        }

        local_pos_pub.publish(pos_target);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}