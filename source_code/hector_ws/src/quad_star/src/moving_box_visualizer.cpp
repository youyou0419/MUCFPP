#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>

class MovingBoxVisualizer {
public:
    MovingBoxVisualizer() {
        nh_.param<std::string>("global_frame", global_frame_, "world");
        model_sub_ = nh_.subscribe("/gazebo/model_states", 10, &MovingBoxVisualizer::modelStatesCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("moving_box_marker", 10);
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        int box_index = getModelIndex(msg->name, "moving_box");
        if (box_index != -1) {
            visualization_msgs::Marker marker;
            createBoxMarker(marker, msg->pose[box_index]);
            marker_pub_.publish(marker);
        }
    }

private:
    int getModelIndex(const std::vector<std::string>& names, const std::string& target) {
        auto it = std::find(names.begin(), names.end(), target);
        return (it != names.end()) ? std::distance(names.begin(), it) : -1;
    }

    void createBoxMarker(visualization_msgs::Marker& marker, const geometry_msgs::Pose& pose) {
        marker.header.frame_id = global_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "moving_box";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = 1.0;  // 与 Gazebo 中定义的尺寸一致
        marker.scale.y = 1.0;
        marker.scale.z = 2.0;
        marker.color.r = 0.0;  // 绿色 (RGB: 0,1,0)
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;  // 不透明
        marker.lifetime = ros::Duration();  // 永不过期
    }

    ros::NodeHandle nh_;
    ros::Subscriber model_sub_;
    ros::Publisher marker_pub_;
    std::string global_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "moving_box_visualizer");
    MovingBoxVisualizer visualizer;
    ros::spin();
    return 0;
}