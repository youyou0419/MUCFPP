#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <map>
#include <string>
#include <deque>
class DroneSafetyController {
public:
    DroneSafetyController() : nh_("~") {
        // 获取无人机列表参数（带默认值）
        std::vector<std::string> drone_names;
        nh_.param("drones", drone_names, {"uav1", "uav2","uav3","uav4"}); // 默认控制uav1和uav2

        // 简单有效性检查
        if (drone_names.empty()) {
            ROS_ERROR("No drones specified! Usage: _drones:='[\"uav1\",\"uav2\"]'");
            ros::shutdown();
            return;
        }

        // 初始化各无人机
        for (const auto& name : drone_names) {
            initDroneController(name);
        }
    }

private:
    struct DroneState {
        // 当前状态
        geometry_msgs::Twist current_vel;
        double roll = 0, pitch = 0, yaw = 0;
        
        // 历史数据（用于振荡检测）
        std::deque<geometry_msgs::Twist> vel_history;
        
        // 发布控制指令
        ros::Publisher cmd_vel_pub;
    };

    ros::NodeHandle nh_;
    std::map<std::string, DroneState> drones_;
    std::vector<ros::Subscriber> orca_subs_;
    std::vector<ros::Subscriber> state_subs_;

    void initDroneController(const std::string& drone_name) {
        // 初始化状态记录
        drones_[drone_name] = DroneState();
        
        // 订阅ORCA输出速度
        orca_subs_.push_back(nh_.subscribe<geometry_msgs::Twist>(
            "/" + drone_name + "/cmd_vel_orca", 1,
            [this, drone_name](const geometry_msgs::Twist::ConstPtr& msg) {
                orcaVelCallback(msg, drone_name);
            }));

        // 订阅无人机状态（获取实际速度和姿态）
        state_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
            "/" + drone_name + "/ground_truth/state", 1,
            [this, drone_name](const nav_msgs::Odometry::ConstPtr& msg) {
                stateCallback(msg, drone_name);
            }));

        // 创建控制指令发布器
        drones_[drone_name].cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>(
            "/" + drone_name + "/cmd_vel", 1);

        ROS_INFO_STREAM("Initialized controller for drone: " << drone_name);
    }

    void orcaVelCallback(const geometry_msgs::Twist::ConstPtr& msg, 
                       const std::string& drone_name) {
        auto& drone = drones_[drone_name];
        
        // // 死锁检测
        // bool is_deadlock = checkDeadlock(drone_name);
        
        // 振荡检测
        bool is_oscillating = checkOscillation(drone_name);
        
        // 决策逻辑
        if (is_oscillating) {
            handleEmergency(drone_name);
        } else {
            // 正常转发ORCA速度
            drone.cmd_vel_pub.publish(*msg);
        }
    }

    void stateCallback(const nav_msgs::Odometry::ConstPtr& msg,
                      const std::string& drone_name) {
        auto& drone = drones_[drone_name];
        
        // 记录当前速度
        drone.current_vel = msg->twist.twist;
        
        // 记录历史数据（保留最近1秒，假设50Hz）
        drone.vel_history.push_back(drone.current_vel);
        if (drone.vel_history.size() > 50) {
            drone.vel_history.pop_front();
        }
        
        // 计算姿态角
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(drone.roll, drone.pitch, drone.yaw);
    }

    bool checkDeadlock(const std::string& drone_name) {
        const auto& drone = drones_[drone_name];
        double speed = sqrt(
            drone.current_vel.linear.x * drone.current_vel.linear.x +
            drone.current_vel.linear.y * drone.current_vel.linear.y);
            
        // 速度低于阈值持续2秒视为死锁
        static std::map<std::string, ros::Time> deadlock_timers;
        if (speed < 0.05) {
            if (deadlock_timers[drone_name].is_zero()) {
                deadlock_timers[drone_name] = ros::Time::now();
            }
            return (ros::Time::now() - deadlock_timers[drone_name]).toSec() > 2.0;
        } else {
            deadlock_timers[drone_name] = ros::Time();
            return false;
        }
    }

    bool checkOscillation(const std::string& drone_name) {
        auto& drone = drones_[drone_name];
        
        // 1. 静止状态直接返回
        if (drone.current_vel.linear.x == 0 && 
            drone.current_vel.linear.y == 0) {
            return false;
        }
    
        // 2. 数据不足时返回
        if (drone.vel_history.size() < 10) return false;
    
        // 3. 改进的姿态变化检测
        static std::map<std::string, std::pair<double, double>> prev_angles;
        double roll_diff = fabs(drone.roll - prev_angles[drone_name].first);
        double pitch_diff = fabs(drone.pitch - prev_angles[drone_name].second);
        prev_angles[drone_name] = {drone.roll, drone.pitch};
    
        // 4. 调整后的阈值
        return (roll_diff > 2.0) || (pitch_diff > 2.0); // 单位：度
    }

    void handleEmergency(const std::string& drone_name) {
        auto& drone = drones_[drone_name];
        
        // 策略1：发送悬停指令
        geometry_msgs::Twist stop_cmd;
        drone.cmd_vel_pub.publish(stop_cmd);
        
        // 策略2：添加随机扰动（可选）
        /*
        geometry_msgs::Twist perturb;
        perturb.linear.x = (rand() % 200 - 100) / 1000.0; // -0.1~0.1
        perturb.linear.y = (rand() % 200 - 100) / 1000.0;
        drone.cmd_vel_pub.publish(perturb);
        */
        
        ROS_WARN_STREAM("Emergency handled for drone: " << drone_name);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "UAVsafetyController");
    DroneSafetyController controller;
    ros::spin();
    return 0;
}