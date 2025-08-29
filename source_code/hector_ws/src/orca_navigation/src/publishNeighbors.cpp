#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>      // 用于 position
#include <geometry_msgs/Vector3.h>    // 用于 linear velocity
#include <string>
#include <map>

class NeighborPublisher {
public:
    NeighborPublisher() : nh_("~") {
        // 参数获取
        nh_.param<std::string>("uav_name", uav_name_, "uav1");
        nh_.param<int>("uav_index", uav_index_, 1); // 默认监听uav1
        nh_.param<std::string>("obstacle_name", obstacle_name_, "moving_box"); // 动态障碍物名称
        
        // 初始化发布器和订阅器
        pub_ = nh_.advertise<gazebo_msgs::ModelStates>("/" + uav_name_ + "/neighbors", 10);
        sub_ = nh_.subscribe("/gazebo/model_states", 1, &NeighborPublisher::modelStatesCallback, this);
        
        ROS_INFO_STREAM("Initialized publisher for UAV: " << uav_name_ 
                     << " listening to index: " << uav_index_);
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        gazebo_msgs::ModelStates neighbors_msg;
        
        // 1. 提取目标无人机信息
        int target_index = -1;
        int obstacle_index = -1;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "uav" + std::to_string(uav_index_)) {
                target_index = i;
                break;
            }
            if (msg->name[i] == obstacle_name_) {
                obstacle_index = i;
            }

        }
        
        if (target_index == -1) {
            ROS_WARN_STREAM_THROTTLE(5, "Target UAV uav" << uav_index_ << " not found!");
            return;
        }

        // 2. 添加目标无人机自身信息（只保留 position 和 linear velocity）
        neighbors_msg.name.push_back(uav_name_);
        
        geometry_msgs::Pose pose;
        pose.position = msg->pose[target_index].position;  // 只保留 position
        neighbors_msg.pose.push_back(pose);

        geometry_msgs::Twist twist;
        twist.linear = msg->twist[target_index].linear;    // 只保留 linear velocity
        neighbors_msg.twist.push_back(twist);

        // 3. 添加其他邻居无人机信息（同样只保留 position 和 linear velocity）
        for (size_t i = 0; i < msg->name.size(); ++i) {
            std::string model_name = msg->name[i];
            
            // 筛选所有uav开头的模型（排除地面站等）
            if (model_name.find("uav") != 0) continue;
            
            // 跳过自身
            if (i == target_index) continue;
            
            neighbors_msg.name.push_back(model_name);

            geometry_msgs::Pose neighbor_pose;
            neighbor_pose.position = msg->pose[i].position;  // 只保留 position
            neighbors_msg.pose.push_back(neighbor_pose);

            geometry_msgs::Twist neighbor_twist;
            neighbor_twist.linear = msg->twist[i].linear;    // 只保留 linear velocity
            neighbors_msg.twist.push_back(neighbor_twist);
        }
        // 4. 添加动态障碍物信息到同一个消息中（使用原始名称）
        if (obstacle_index != -1) {
            neighbors_msg.name.push_back(obstacle_name_); // 使用障碍物原始名称
            
            geometry_msgs::Pose obstacle_pose;
            obstacle_pose.position = msg->pose[obstacle_index].position;
            neighbors_msg.pose.push_back(obstacle_pose);

            geometry_msgs::Twist obstacle_twist;
            obstacle_twist.linear = msg->twist[obstacle_index].linear;
            neighbors_msg.twist.push_back(obstacle_twist);
        } else {
            ROS_WARN_STREAM_THROTTLE(5, "Dynamic obstacle " << obstacle_name_ << " not found in Gazebo!");
        }

        // 4. 发布邻居信息
        pub_.publish(neighbors_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::string uav_name_;
    int uav_index_;
    std::string obstacle_name_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "neighbor_publisher");
    NeighborPublisher np;
    ros::spin();
    return 0;
}