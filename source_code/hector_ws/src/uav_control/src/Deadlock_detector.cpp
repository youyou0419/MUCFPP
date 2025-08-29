#include <ros/ros.h>
#include <orca_navigation/BLockCondition.h>
#include <uav_control/DeadlockInfo.h>  // 新增消息类型
#include <std_msgs/Bool.h>
#include <map>
#include <vector>
#include <algorithm>

class DeadlockDetector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber block_sub_;
    ros::Publisher deadlock_pub_;  // 新增：公共deadlock信息发布器
    std::map<std::string, ros::Publisher> stop_pubs_; // 存储每个无人机的Publisher
    
    std::map<std::pair<std::string, std::string>, int> block_counter_;
    ros::Time last_clear_time_;
    
    const int THRESHOLD = 4;
    const double CLEAR_INTERVAL = 5.0;

public:
    DeadlockDetector() : last_clear_time_(ros::Time::now()) {
        block_sub_ = nh_.subscribe("/blockProcess", 10, &DeadlockDetector::blockCallback, this);
        deadlock_pub_ = nh_.advertise<uav_control::DeadlockInfo>("/deadlock_info", 1); // 初始化公共发布器
    }

    // 新增：发布deadlock信息到公共topic
    void publishDeadlockInfo(const std::string& blocker_id, 
        const std::string& blocked_id,
        bool is_bilateral) {
        uav_control::DeadlockInfo msg;
        msg.blocker_id = blocker_id;
        msg.blocked_id = blocked_id;
        msg.is_bilateral = is_bilateral;
        deadlock_pub_.publish(msg);

        ROS_INFO("[DeadlockInfo] Published %s between %s and %s",
        is_bilateral ? "BILATERAL deadlock" : "UNILATERAL obstruction",
        blocker_id.c_str(), blocked_id.c_str());
}

    void blockCallback(const orca_navigation::BLockCondition::ConstPtr& msg) {
        // 检查blocker_id前三个字符是否为"uav"，不是则忽略
        if (msg->blocker_id.compare(0, 3, "uav") != 0) {
            ROS_DEBUG("Ignoring non-UAV blocker: %s", msg->blocker_id.c_str());
            return;
        }

        if ((ros::Time::now() - last_clear_time_).toSec() > CLEAR_INTERVAL) {
            block_counter_.clear();
            last_clear_time_ = ros::Time::now();
            ROS_DEBUG("Cleared block counter");
        }
    
        std::pair<std::string, std::string> block_pair(msg->blocker_id, msg->blocked_id);
        block_counter_[block_pair]++;
        
        // 检查单向阻塞是否超过阈值
        if (block_counter_[block_pair] >= THRESHOLD) {
            std::pair<std::string, std::string> reverse_pair(msg->blocked_id, msg->blocker_id);
            auto reverse_it = block_counter_.find(reverse_pair);
            
            // 如果是单向阻塞
            if (reverse_it == block_counter_.end() || reverse_it->second < THRESHOLD/4) {
                ROS_WARN("[Obstruction] UAV %s is blocked by %s (count: %d)", 
                        msg->blocked_id.c_str(), msg->blocker_id.c_str(),
                        block_counter_[block_pair]);
                
                // 发送停止命令（保持原样）
                sendStopCommand(msg->blocked_id, true);
                
                // 新增：发布阻塞信息到公共topic
                publishDeadlockInfo(msg->blocker_id, msg->blocked_id, false); // false表示单向
                
                block_counter_[block_pair] = 0;
                return;
            }
            
            // 双向死锁检测
            if (reverse_it->second >= THRESHOLD) {
                ROS_WARN("[Deadlock] Detected between %s and %s", 
                        msg->blocker_id.c_str(), msg->blocked_id.c_str());
                // 发送停止命令（保持原样）
                sendStopCommand(msg->blocker_id, true);
                sendStopCommand(msg->blocked_id, true);
                
                // 新增：发布死锁信息到公共topic
                publishDeadlockInfo(msg->blocker_id, msg->blocked_id, true); // true表示双向
                block_counter_[block_pair] = 0;
                block_counter_[reverse_pair] = 0;
            }
        }
    }

    // 保持原样的sendStopCommand函数
    void sendStopCommand(const std::string& uav_name, bool stop) {
        // 检查是否已有该无人机的Publisher，没有则创建
        if (stop_pubs_.find(uav_name) == stop_pubs_.end()) {
            stop_pubs_[uav_name] = nh_.advertise<std_msgs::Bool>(
                "/" + uav_name + "/needstop", 1, true);
            ROS_DEBUG("Created publisher for %s", uav_name.c_str());
            
            // 等待连接建立
            ros::Time start = ros::Time::now();
            while (stop_pubs_[uav_name].getNumSubscribers() == 0 && 
                  (ros::Time::now() - start).toSec() < 1.0) {
                ros::Duration(0.1).sleep();
            }
        }
        
        std_msgs::Bool msg;
        msg.data = stop;
        stop_pubs_[uav_name].publish(msg);
        
        ROS_INFO("[Control] Sent %s to %s (subscribers: %d)", 
                stop ? "STOP" : "RESUME", 
                uav_name.c_str(),
                stop_pubs_[uav_name].getNumSubscribers());
    }

    void run() {
        ros::Rate rate(50);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Deadlock_detector");
    DeadlockDetector detector;
    detector.run();
    return 0;
}