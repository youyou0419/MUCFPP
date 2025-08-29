#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <string>
#include <algorithm>
#include <cmath>

// 控制参数
#define SPEED 2.0f
#define MAX_SPEED 2.0f
#define POSITION_TOLERANCE 0.1
#define EPSILON 0.01

struct UAVState {
    geometry_msgs::Point position;
    double yaw;
};

class UAVController {
private:
    ros::NodeHandle nh_private_;  // 用于参数读取（私有空间）
    ros::NodeHandle nh_global_;   // 用于全局话题通信
    ros::Subscriber model_states_sub_;
    ros::Subscriber target_sub_;
    ros::Publisher cmd_vel_pub_;
    
    UAVState current_state_;
    geometry_msgs::Point target_position_;
    std::string model_name_;
    bool initialized_ = false;

    // 方向系数计算
    double coeff(double a, double b) {
        if(fabs(a-b) < EPSILON) return 0;
        return a < b ? 1.0 : -1.0;
    }

    // 四元数转偏航角
    double quaternionToYaw(const geometry_msgs::Quaternion& q) {
        return atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z));
    }

public:
    UAVController():
    nh_private_("~")// 私有命名空间用于参数
       // 全局命名空间用于话题
{       
        // 从参数服务器获取模型名称
        if (!nh_private_.getParam("model_name", model_name_)) {
            model_name_ = "uav1"; // 默认值作为最后保障
            ROS_WARN("Using default model name: %s", model_name_.c_str());
        }
        
        // 初始化话题
        cmd_vel_pub_ = nh_private_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        target_sub_ = nh_private_.subscribe("target_tracking_pose", 1, &UAVController::targetCallback, this);
        model_states_sub_ = nh_private_.subscribe("/gazebo/model_states", 1, &UAVController::modelStatesCallback, this);
        
        ROS_INFO("Controller initialized for UAV: %s", model_name_.c_str());
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        // 查找对应模型
        auto it = std::find(msg->name.begin(), msg->name.end(), model_name_);
        if(it != msg->name.end()) {
            size_t index = it - msg->name.begin();
            current_state_.position = msg->pose[index].position;
            current_state_.yaw = quaternionToYaw(msg->pose[index].orientation);
            
            if(!initialized_) {
                target_position_ = current_state_.position;
                initialized_ = true;
                ROS_INFO("%s: Initial target set to current position", model_name_.c_str());
            }
        }
    }

    void targetCallback(const geometry_msgs::Vector3& msg) {
        target_position_.x = msg.x;
        target_position_.y = msg.y;
        target_position_.z = msg.z;
        // ROS_INFO("%s: New target received [%.2f, %.2f, %.2f]", 
        //         model_name_.c_str(), msg.x, msg.y, msg.z);
    }

    void controlLoop() {
        if(!initialized_) return;

        // 计算距离差
        double dx = target_position_.x - current_state_.position.x;
        double dy = target_position_.y - current_state_.position.y;
        double dz = target_position_.z - current_state_.position.z;
        
        // 到达判定
        if(fabs(dx) < POSITION_TOLERANCE && 
           fabs(dy) < POSITION_TOLERANCE && 
           fabs(dz) < POSITION_TOLERANCE) {
            cmd_vel_pub_.publish(geometry_msgs::Twist());
            // ROS_INFO_THROTTLE(1.0, "%s: Reached target position", model_name_.c_str());
            return;
        }

        geometry_msgs::Twist cmd;

        // 速度计算（保持原始逻辑）
        cmd.linear.x = SPEED * fabs(dx) * coeff(current_state_.position.x, target_position_.x);
        cmd.linear.y = SPEED * fabs(dy) * coeff(current_state_.position.y, target_position_.y);
        cmd.linear.z = SPEED * fabs(dz) * coeff(current_state_.position.z, target_position_.z);

        // 水平速度限幅
        double horiz_speed = sqrt(cmd.linear.x*cmd.linear.x + cmd.linear.y*cmd.linear.y);
        if(horiz_speed > MAX_SPEED) {
            cmd.linear.x *= MAX_SPEED / horiz_speed;
            cmd.linear.y *= MAX_SPEED / horiz_speed;
        }

        // 偏航控制（保持原始简单控制）

        // cmd.angular.z = -current_state_.yaw/8;
        cmd.angular.z = 0;
        cmd.angular.y = 0;
        cmd.angular.x = 0;


        cmd_vel_pub_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "singleGoPose");
    UAVController controller;
    ros::Rate rate(50);
    
    while(ros::ok()) {
        ros::spinOnce();
        controller.controlLoop();
        rate.sleep();
    }
    return 0;
}