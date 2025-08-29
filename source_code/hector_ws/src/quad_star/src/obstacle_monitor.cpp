#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tuple>
#include <unordered_map>

class ObstacleMonitor {
public:
    ObstacleMonitor() : nh_("~") {
        // 初始化尺寸映射表
        obstacle_dimensions_ = {
            {"wall_1",     {0.2, 8.0, 2.0}},
            {"wall_2",     {0.2, 8.0, 2.0}},
            {"north_wall", {20.0, 0.2, 2.0}},
            {"south_wall", {20.0, 0.2, 2.0}},
            {"east_wall",  {0.2, 20.4, 2.0}},
            {"west_wall",  {0.2, 20.4, 2.0}}
        };

        // 参数配置
        nh_.getParam("obstacle_models", obstacle_models_);
        if(obstacle_models_.empty()) {
            ROS_WARN("No obstacle models specified!");
        }

        // 初始化发布器
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 10);
        
        // 订阅Gazebo模型状态
        model_sub_ = nh_.subscribe("/gazebo/model_states", 10, 
                                 &ObstacleMonitor::modelStatesCallback, this);
    }

private:
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        static tf2_ros::TransformBroadcaster br;
        visualization_msgs::MarkerArray markers;
        
        // 先添加删除指令
        visualization_msgs::Marker clear_marker;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear_marker);

        for(size_t i = 0; i < msg->name.size(); ++i) {
            if(isObstacleModel(msg->name[i])) {
                // 发布TF
                publishTransform(msg->name[i], msg->pose[i]);
                
                // 创建带精确尺寸的可视化标记
                visualization_msgs::Marker marker = createMarker(msg->name[i], msg->pose[i], i);
                if(!marker.header.frame_id.empty()) { // 有效标记
                    markers.markers.push_back(marker);
                }
            }
        }
        
        marker_pub_.publish(markers);
    }

    bool isObstacleModel(const std::string& model_name) {
        return obstacle_dimensions_.find(model_name) != obstacle_dimensions_.end();
    }

    void publishTransform(const std::string& model_name, 
                        const geometry_msgs::Pose& pose) {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "world";
        transform.child_frame_id = "obstacle_" + model_name;
        transform.transform.translation.x = pose.position.x;
        transform.transform.translation.y = pose.position.y;
        transform.transform.translation.z = pose.position.z;
        transform.transform.rotation = pose.orientation;
        br.sendTransform(transform);
    }

    visualization_msgs::Marker createMarker(const std::string& model_name,
                                          const geometry_msgs::Pose& pose,
                                          int id) 
    {
        visualization_msgs::Marker marker;
        auto it = obstacle_dimensions_.find(model_name);
        if(it == obstacle_dimensions_.end()) {
            ROS_WARN_STREAM("Undefined obstacle dimensions for: " << model_name);
            return marker;
        }

        // 从映射表获取精确尺寸
        auto [scale_x, scale_y, scale_z] = it->second;
        
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacles";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = scale_x;
        marker.scale.y = scale_y;
        marker.scale.z = scale_z;
        marker.color.r = 0.3;
        marker.color.g = 0.3;
        marker.color.b = 0.3;
        marker.color.a = 0.7;
        marker.lifetime = ros::Duration(0.5);
        return marker;
    }

    ros::NodeHandle nh_;
    ros::Subscriber model_sub_;
    ros::Publisher marker_pub_;
    tf2_ros::TransformBroadcaster br;
    std::vector<std::string> obstacle_models_;
    std::unordered_map<std::string, std::tuple<double, double, double>> obstacle_dimensions_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_monitor");
    ObstacleMonitor monitor;
    ros::spin();
    return 0;
}