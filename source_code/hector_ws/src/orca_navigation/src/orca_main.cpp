#include <ros/ros.h>
#include "Agent.h"
#include "motionUtilities.hpp"
#include "Vector2.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <stdlib.h>
#include <orca_navigation/BLockCondition.h>

std::map<std::string, std::vector<RVO::Vector2>> wall_vertices = {
    {"wall_1", {  // 修正为逆时针
        RVO::Vector2(-0.1f, 1.0f),
        RVO::Vector2(0.1f, 1.0f),
        RVO::Vector2(0.1f, 10.0f),
        RVO::Vector2(-0.1f, 10.0f)
    }},
    {"wall_2", {  // 修正为逆时针
        RVO::Vector2(-0.1f, -1.0f),
        RVO::Vector2(-0.1f, -10.0f),
        RVO::Vector2(0.1f, -10.0f),
        RVO::Vector2(0.1f, 1.0f)
    }},
    {"north_wall", {  // 修正为逆时针
        RVO::Vector2(-10.0f, 10.0f),
        RVO::Vector2(10.0f, 10.0f),
        RVO::Vector2(10.0f, 10.2f),
        RVO::Vector2(-10.0f, 10.2f)
    }},
    {"south_wall", {  // 修正为逆时针
        RVO::Vector2(-10.0f, -10.2f),
        RVO::Vector2(10.0f, -10.2f),
        RVO::Vector2(10.0f, -10.0f),
        RVO::Vector2(-10.0f, -10.0f)
    }},
    {"east_wall", {  // 修正为逆时针
        RVO::Vector2(10.0f, -10.2f),
        RVO::Vector2(10.2f, -10.2f),
        RVO::Vector2(10.2f, 10.2f),
        RVO::Vector2(10.0f, 10.2f)
    }},
    {"west_wall", {  // 修正为逆时针
        RVO::Vector2(-10.2f, -10.2f),
        RVO::Vector2(-10.0f, -10.2f),
        RVO::Vector2(-10.0f, 10.2f),
        RVO::Vector2(-10.2f, 10.2f)
    }}
};
ros::Publisher *pubPtr;
ros::Publisher *block_pubPtr;
RVO::Agent *quad;

// RVO::Agent quad("uav",COLLISION_RADIUS);

gazebo_msgs::ModelStates neighbors;
bool ready=false;
bool emergency_stop_=false; // 新增：紧急停止标志
void stopCallback(const std_msgs::Bool::ConstPtr& msg) {
    emergency_stop_ = msg->data;
    if (emergency_stop_) {
        geometry_msgs::Twist stop_vel;
        stop_vel.linear.x = 0;
        stop_vel.linear.y = 0;
        stop_vel.linear.z = 0;
        pubPtr->publish(stop_vel);
        ROS_WARN("[Safety] Received STOP command! UAV will halt.");
    } else {
        ROS_INFO("[Safety] STOP command released. Resuming normal operation.");
    }
}
// 在初始化函数中添加所有墙体障碍物（只需运行一次）
void initWalls() {
    for (const auto& wall : wall_vertices) {
        quad->addObstacle(wall.second);  // 直接添加预定义的顶点数据
    }
}
void velCallback(const geometry_msgs::Twist &msg)
{
    // 系统就绪检查
    if(!ready) {
        ROS_INFO("[status check] unready!! direct exec the pre vel");
        // pubPtr->publish(msg);
        return;
    }
    if (emergency_stop_) {
        // geometry_msgs::Twist stop_vel;
        // stop_vel.linear.x = 0;
        // stop_vel.linear.y = 0;
        // stop_vel.linear.z = 0;
        // pubPtr->publish(stop_vel);
        return;
    }

    // // 显示接收到的原始速度
    // ROS_INFO("[origin vel] linear: x=%.2f y=%.2f z=%.2f", 
    //          msg.linear.x, msg.linear.y, msg.linear.z);

    // 核心处理流程
    try {
        // 准备步骤
        quad->prepareStep(neighbors, msg);
        initWalls();
        // 速度计算
        std::string blocker_id=quad->computeNewVelocity();
        if (!blocker_id.empty())
            {orca_navigation::BLockCondition blockmsg;
                blockmsg.blocker_id=blocker_id;
                blockmsg.blocked_id=quad->id_;
                blockmsg.stamp=ros::Time::now();
                block_pubPtr->publish(blockmsg);}
        // 检查ORCA输出速度是否为零
        const double MIN_SPEED = 0.01; // 零速阈值
        if (fabs(quad->newVelocity_.x_) < MIN_SPEED && 
            fabs(quad->newVelocity_.y_) < MIN_SPEED) 
        {
            // 对原始msg添加随机扰动（±0.2m/s范围）
            geometry_msgs::Twist perturbed_msg = msg;
            perturbed_msg.linear.x += (rand() % 200 - 100) * 0.001;  // ±0.1m/s
            perturbed_msg.linear.y += (rand() % 200 - 100) * 0.001;  // ±0.1m/s
            
            // ROS_WARN("[Deadlock] Adding perturbation to input velocity (dx=%.2f, dy=%.2f)",
            //         perturbed_msg.linear.x - msg.linear.x,
            //         perturbed_msg.linear.y - msg.linear.y);

            // 用扰动后的速度重新计算ORCA
            quad->prepareStep(neighbors, perturbed_msg);
            std::string blocker_id=quad->computeNewVelocity();
            if (!blocker_id.empty())
            {orca_navigation::BLockCondition blockmsg;
                blockmsg.blocker_id=blocker_id;
                blockmsg.blocked_id=quad->id_;
                blockmsg.stamp=ros::Time::now();
                block_pubPtr->publish(blockmsg);}
        }
        // 构建新速度命令
        geometry_msgs::Twist newVel = msg;
        newVel.linear.x = quad->newVelocity_.x_;
        newVel.linear.y = quad->newVelocity_.y_;

        // 显示计算结果
        // ROS_INFO("[vel_corect] new velocity: x=%.2f y=%.2f ",
        //         newVel.linear.x, newVel.linear.y);

        // ===== 简化版死锁处理 =====
        // const double MIN_SPEED = 0.05; // 速度阈值ros::Subscriber sub = nh.subscribe("cmd_vel_pref", 1, &velCallback);
        // if (fabs(newVel.linear.x) < MIN_SPEED && 
        //     fabs(newVel.linear.y) < MIN_SPEED) 
        // {
        //     // 添加简单随机扰动（±0.1m/s）
        //     newVel.linear.x += (rand() % 200 - 100) * 0.01; 
        //     newVel.linear.y += (rand() % 200 - 100) * 0.01;
        //     ROS_WARN("[Deadlock] Applying random perturbation");
        // }
        // 发布结果
        pubPtr->publish(newVel);
        // ROS_INFO("[new vel] fixed orca veocity");

    } catch (const std::exception& e) {
        ROS_INFO("[异常处理] 计算错误: %s (已回退原始速度)", e.what());
        pubPtr->publish(msg);
    }
}



void neighborCallback(const gazebo_msgs::ModelStates &msg) {
    ready = true;                  // 标记数据就绪
    neighbors = msg;               // 存储邻居信息
}

int main(int argc, char* argv[])
{
    srand(time(NULL));//初始化时间种子
    ros::init(argc, argv, "orac_main");
    ros::NodeHandle nh("~");
    std::string uav_name;
    nh.getParam("uav_name", uav_name);
    quad = new RVO::Agent(uav_name, COLLISION_RADIUS);  // 动态名称

    initWalls();
    ros::Subscriber sub = nh.subscribe("cmd_vel_pref", 1, &velCallback);
    ros::Subscriber sub2= nh.subscribe("neighbors",1,&neighborCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber stop_sub = nh.subscribe("needstop", 1, &stopCallback);  // 新增停止订阅
    ros::Publisher block_pub = nh.advertise<orca_navigation::BLockCondition>("/blockProcess", 20);
    block_pubPtr = &block_pub;
    pubPtr = &pub;
    ros::Rate rate(50);
    while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    delete quad;
    return 0;
}