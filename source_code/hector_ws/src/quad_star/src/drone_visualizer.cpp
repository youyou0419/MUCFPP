#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/MarkerArray.h>

class DroneVisualizer {
public:
    DroneVisualizer() {
        nh_.param<std::string>("global_frame", global_frame_, "world");
        model_sub_ = nh_.subscribe("/gazebo/model_states", 10, &DroneVisualizer::modelStatesCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_markers", 10);
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        visualization_msgs::MarkerArray markers;
        
        // 无人机模型可视化
        int uav_index = getModelIndex(msg->name, "uav1");
        if (uav_index != -1) {
            createDroneMarker(markers.markers, msg->pose[uav_index]);
        }

        marker_pub_.publish(markers);
    }

private:
    // 获取模型索引
    int getModelIndex(const std::vector<std::string>& names, const std::string& target) {
        auto it = std::find(names.begin(), names.end(), target);
        return (it != names.end()) ? std::distance(names.begin(), it) : -1;
    }

    // 创建无人机Marker
    void createDroneMarker(std::vector<visualization_msgs::Marker>& markers, 
                         const geometry_msgs::Pose& pose) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = global_frame_;
        marker.ns = "uav";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = "package://hector_quadrotor_description/meshes/quadrotor/quadrotor_base.dae";
        marker.pose = pose;
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 0.9;  // 珊瑚色
        marker.color.g = 0.3;
        marker.color.b = 0.2;
        markers.push_back(marker);
    }

    ros::NodeHandle nh_;
    ros::Subscriber model_sub_;
    ros::Publisher marker_pub_;
    std::string global_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_visualizer");
    DroneVisualizer visualizer;
    ros::spin();
    return 0;
}