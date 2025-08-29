#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <string>

class UAVPathTracker {
private:      
    ros::NodeHandle private_nh;    // 私有句柄(用于参数和私有话题)
    ros::Subscriber model_states_sub;
    ros::Subscriber path_sub;
    ros::Publisher target_pub;
    
    // 无人机状态
    geometry_msgs::Pose current_pose;
    bool pose_initialized;
    bool path_received;
    
    // 跟踪参数
    size_t current_waypoint;
    double position_tolerance;
    std::string uav_name;
    std::vector<geometry_msgs::Point> path;

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == uav_name) {
                current_pose = msg->pose[i];
                if (!pose_initialized) {
                    ROS_INFO("%s position initialized!", uav_name.c_str());
                    pose_initialized = true;
                }
                break;
            }
        }
    }

    void pathCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        path.clear();
        for (const auto& pose : msg->poses) {
            geometry_msgs::Point point; 
            point.x = pose.position.x;
            point.y = pose.position.y;
            point.z = pose.position.z;
            path.push_back(point);
        }
        
        current_waypoint = 0;
        path_received = true;
        ROS_INFO("%s: Received new path with %zu waypoints", 
                uav_name.c_str(), path.size());
    }

    void controlLoop() {
        if (!pose_initialized || !path_received) return;

        const geometry_msgs::Point& target = path[current_waypoint];
        double dx = target.x - current_pose.position.x;
        double dy = target.y - current_pose.position.y;
        double dz = target.z - current_pose.position.z;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);

        if (distance < position_tolerance) {
            if (current_waypoint < path.size() - 1) {
                current_waypoint++;
                ROS_INFO("%s: Reached waypoint %zu/%zu", 
                        uav_name.c_str(), current_waypoint, path.size());
            } else {
                ROS_INFO("%s: Completed entire path!", uav_name.c_str());
                path_received = false;
            }
        }

        target_pub.publish(path[current_waypoint]);
    }

public:
    UAVPathTracker() : 
        private_nh("~"),  // 初始化私有句柄
        pose_initialized(false),
        path_received(false),
        current_waypoint(0),
        position_tolerance(0.2)
    {
        // 从私有参数服务器获取参数
        private_nh.param<std::string>("uav_name", uav_name, "uav1");
        private_nh.param<double>("position_tolerance", position_tolerance, 0.3);
        
        // 使用公有句柄订阅全局话题
        model_states_sub = private_nh.subscribe(
            "/gazebo/model_states", 1, &UAVPathTracker::modelStatesCallback, this);
            
        // 使用私有句柄订阅私有话题
        path_sub = private_nh.subscribe(
            "path", 1, &UAVPathTracker::pathCallback, this);
            
        // 使用私有句柄发布私有话题
        target_pub = private_nh.advertise<geometry_msgs::Point>(
            "target_pose", 1);

        ROS_INFO("%s path tracker ready, waiting for path...", uav_name.c_str());
    }

    void run() {
        ros::Rate rate(100);
        while (ros::ok()) {
            ros::spinOnce();
            controlLoop();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_path_tracker");
    UAVPathTracker tracker;
    tracker.run();
    return 0;
}