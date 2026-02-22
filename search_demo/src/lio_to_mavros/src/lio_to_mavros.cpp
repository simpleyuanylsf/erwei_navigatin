
// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <Eigen/Eigen>
// #include <cmath>
// #include <queue>
 
// Eigen::Vector3d p_lidar_body, p_enu;
// Eigen::Quaterniond q_mav;
// Eigen::Quaterniond q_px4_odom;

// // 【修改1】声明全局时间戳变量，否则编译报错
// ros::Time lidar_timestamp; 

// class SlidingWindowAverage {
// public:
//     SlidingWindowAverage(int windowSize) : windowSize(windowSize), windowSum(0.0) {}

//     double addData(double newData) {
//         if(!dataQueue.empty()&&fabs(newData-dataQueue.back())>0.01){
//             dataQueue = std::queue<double>();
//             windowSum = 0.0;
//             dataQueue.push(newData);
//             windowSum += newData;
//         }
//         else{            
//             dataQueue.push(newData);
//             windowSum += newData;
//         }

//         if (dataQueue.size() > windowSize) {
//             windowSum -= dataQueue.front();
//             dataQueue.pop();
//         }
//         windowAvg = windowSum / dataQueue.size();
//         return windowAvg;
//     }

//     int get_size(){
//         return dataQueue.size();
//     }

//     double get_avg(){
//         return windowAvg;
//     }

// private:
//     int windowSize;
//     double windowSum;
//     double windowAvg;
//     std::queue<double> dataQueue;
// };

// int windowSize = 8;
// SlidingWindowAverage swa=SlidingWindowAverage(windowSize);

// double fromQuaternion2yaw(Eigen::Quaterniond q)
// {
//   double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
//   return yaw;
// }

// void lio_callback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     p_lidar_body = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
//     q_mav = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    
//     // 【修改2】正确获取并保存 Fast-LIO 的原始时间戳
//     lidar_timestamp = msg->header.stamp;
// }
 
// void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     q_px4_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
//     swa.addData(fromQuaternion2yaw(q_px4_odom));
// } 

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "lio_to_mavros");
//     ros::NodeHandle nh("~");
 
//     ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, lio_callback);
//     ros::Subscriber px4_odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 5, px4_odom_callback);
 
//     ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
 
//     ros::Rate rate(50.0); // 建议提高到 30Hz-50Hz，20Hz 对 Fast-LIO 略低
 
//     bool init_flag = 0;
//     float init_yaw = 0.0;
//     Eigen::Quaterniond init_q;

//     while(ros::ok()){
//         // 初始化逻辑
//         if(swa.get_size()==windowSize && !init_flag){
//             init_yaw = swa.get_avg();
//             init_flag = 1;
//             // 构建只包含 Yaw 旋转的四元数
//             init_q = Eigen::AngleAxisd(init_yaw, Eigen::Vector3d::UnitZ());
//             ROS_INFO("Initialization Complete. Yaw Offset: %f rad", init_yaw);
//         }

//         if(init_flag){
//             geometry_msgs::PoseStamped vision;
            
//             // 1. 位置旋转 (你原来的代码是对的)
//             p_enu = init_q * p_lidar_body;
            
//             // 2. 【核心修改】姿态旋转 (必须加上这一步！)
//             // 即使禁用了罗盘，为了保证坐标系严格对齐，这里必须旋转
//             Eigen::Quaterniond q_enu = init_q * q_mav; 

//             vision.pose.position.x = p_enu[0];
//             vision.pose.position.y = p_enu[1];
//             vision.pose.position.z = p_enu[2];
    
//             vision.pose.orientation.x = q_enu.x();
//             vision.pose.orientation.y = q_enu.y();
//             vision.pose.orientation.z = q_enu.z();
//             vision.pose.orientation.w = q_enu.w();
    
//             // 3. 【修改3】使用原始时间戳，减少 EKF 滞后
//             vision.header.stamp = lidar_timestamp;
//             vision.header.frame_id = "map"; // 建议加上 Frame ID
            
//             vision_pub.publish(vision);
//         }
 
//         ros::spinOnce();
//         rate.sleep();
//     }
 
//     return 0;
// }
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>
#include <queue>

// 全局变量
Eigen::Vector3d p_lidar_body, p_enu;
Eigen::Quaterniond q_mav;
Eigen::Quaterniond q_px4_odom;
ros::Time lidar_timestamp; // 保存原始时间戳

// 滑动窗口类 (保持不变)
class SlidingWindowAverage {
public:
    SlidingWindowAverage(int windowSize) : windowSize(windowSize), windowSum(0.0) {}

    double addData(double newData) {
        if(!dataQueue.empty() && fabs(newData - dataQueue.back()) > 0.01){
            // 如果跳变太大，重置队列（防止角度跳变 3.14 -> -3.14 导致的平均值错误）
            // 注意：简单的平均无法处理 -PI 到 PI 的跳变，这里暂时沿用你的逻辑
            // 建议：实际工程中推荐使用四元数插值或处理角度回绕
            dataQueue = std::queue<double>();
            windowSum = 0.0;
            dataQueue.push(newData);
            windowSum += newData;
        }
        else{            
            dataQueue.push(newData);
            windowSum += newData;
        }

        if (dataQueue.size() > windowSize) {
            windowSum -= dataQueue.front();
            dataQueue.pop();
        }
        windowAvg = windowSum / dataQueue.size();
        return windowAvg;
    }

    int get_size(){ return dataQueue.size(); }
    double get_avg(){ return windowAvg; }

private:
    int windowSize;
    double windowSum;
    double windowAvg;
    std::queue<double> dataQueue;
};

int windowSize = 10;
SlidingWindowAverage swa(windowSize);

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
    return atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
}

void lio_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    p_lidar_body = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    q_mav = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    
    // 【关键】获取并保存 Fast-LIO 的原始时间戳，减少 EKF 延迟影响
    lidar_timestamp = msg->header.stamp;
}
 
void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    q_px4_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    swa.addData(fromQuaternion2yaw(q_px4_odom));
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_to_mavros");
    ros::NodeHandle nh("~");
 
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 10, lio_callback); // 队列不需要太大，用最新的
    ros::Subscriber px4_odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, px4_odom_callback);
 
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
 
    ros::Rate rate(30.0); // 30Hz 足够
 
    bool init_flag = false;
    float init_yaw = 0.0;
    Eigen::Quaterniond init_q;

    // 等待一小会儿确保有数据
    ros::Duration(1.0).sleep();

    while(ros::ok()){
        // 初始化逻辑：对齐 LIO 和 PX4 的初始航向
        if(swa.get_size() >= windowSize && !init_flag){
            init_yaw = swa.get_avg();
            init_flag = true;
            // 构建只包含 Yaw 旋转的四元数 (LIO Frame -> ENU Frame)
            init_q = Eigen::AngleAxisd(init_yaw, Eigen::Vector3d::UnitZ());
            ROS_INFO("LIO-MAVROS Align Initialized. Yaw Offset: %.2f rad", init_yaw);
        }

        // 只有当时间戳非零（收到过 LIO 数据）且初始化完成后才发布
        if(init_flag && !lidar_timestamp.isZero()){
            geometry_msgs::PoseStamped vision;
            
            // 1. 位置旋转
            p_enu = init_q * p_lidar_body;
            
            // 2. 姿态旋转 (修复了这里)
            Eigen::Quaterniond q_enu = init_q * q_mav; 

            vision.pose.position.x = p_enu[0];
            vision.pose.position.y = p_enu[1];
            vision.pose.position.z = p_enu[2];
    
            vision.pose.orientation.x = q_enu.x();
            vision.pose.orientation.y = q_enu.y();
            vision.pose.orientation.z = q_enu.z();
            vision.pose.orientation.w = q_enu.w();
    
            // 3. 填充 Header (修复了这里)
            vision.header.stamp = lidar_timestamp; // 使用原始时间戳
            vision.header.frame_id = "map";        // 必须设置 Frame ID，通常为 map 或 odom
            
            vision_pub.publish(vision);
        }
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}