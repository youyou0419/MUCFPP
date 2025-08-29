#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/MarkerArray.h>
#include <regex>
#include <unordered_map>
#include <vector>

class UnlimitedDroneVisualizer {
public:
    UnlimitedDroneVisualizer() : nh_("~"), color_index_(0) {
        // 参数配置
        nh_.param<std::string>("global_frame", global_frame_, "world");
        nh_.param<std::string>("model_prefix", model_prefix_, "uav");
        nh_.param<std::string>("mesh_resource", mesh_resource_, 
            "package://hector_quadrotor_description/meshes/quadrotor/quadrotor_base.dae");

        // 统一颜色配置（珊瑚色）
        uniform_color_.r = 0.9;
        uniform_color_.g = 0.3;
        uniform_color_.b = 0.2;
        uniform_color_.a = 1.0;

        // 初始化轨迹颜色列表
        trail_colors_ = {
            createColor(0.0, 0.0, 0.8, 0.9),   // 深蓝色
            createColor(0.0, 0.6, 0.0, 0.9),   // 深绿色
            createColor(0.7, 0.5, 0.0, 0.9),   // 深橙色
            createColor(0.5, 0.0, 0.5, 0.9),   // 深紫色
            createColor(0.0, 0.5, 0.5, 0.9),    // 深青色
        };

        // 订阅发布设置
        model_sub_ = nh_.subscribe("/gazebo/model_states", 10, 
                                 &UnlimitedDroneVisualizer::modelStatesCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_markers", 10);
        trail_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("drone_trails", 10);
    }

private:
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::MarkerArray trails;
        std::regex re(model_prefix_ + "\\d+");

        for(size_t i = 0; i < msg->name.size(); ++i) {
            if(std::regex_match(msg->name[i], re)) {
                // 处理无人机模型显示
                visualization_msgs::Marker marker;
                marker.header.frame_id = global_frame_;
                marker.header.stamp = ros::Time::now();
                marker.ns = msg->name[i];
                marker.id = 0;
                marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                marker.mesh_resource = mesh_resource_;
                marker.pose = msg->pose[i];
                marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
                marker.color = uniform_color_;
                markers.markers.push_back(marker);

                // 为新发现的无人机分配颜色
                if(color_map_.find(msg->name[i]) == color_map_.end()) {
                    color_map_[msg->name[i]] = trail_colors_[color_index_ % trail_colors_.size()];
                    color_index_++;
                }

                // 处理轨迹显示（无上限）
                geometry_msgs::Point pt;
                pt.x = msg->pose[i].position.x;
                pt.y = msg->pose[i].position.y;
                pt.z = msg->pose[i].position.z;

                trail_map_[msg->name[i]].push_back(pt);  // 持续存储所有历史点

                // 创建轨迹Marker
                visualization_msgs::Marker trail_marker;
                trail_marker.header.frame_id = global_frame_;
                trail_marker.header.stamp = ros::Time::now();
                trail_marker.ns = msg->name[i] + "_trail";  // 使用无人机名称+_trail作为命名空间
                trail_marker.id = 0;  // 固定ID，因为每个无人机有自己的命名空间
                trail_marker.type = visualization_msgs::Marker::LINE_STRIP;
                trail_marker.action = visualization_msgs::Marker::ADD;
                
                // 设置轨迹颜色
                trail_marker.color = color_map_[msg->name[i]];
                trail_marker.scale.x = 0.05;  // 线宽
                
                // 添加所有历史点到轨迹
                trail_marker.points = trail_map_[msg->name[i]];
                trail_marker.lifetime = ros::Duration();  // 永久显示

                trails.markers.push_back(trail_marker);
            }
        }

        // 发布标记
        marker_pub_.publish(markers);
        trail_pub_.publish(trails);
    }

    // 辅助函数：颜色转换
    std_msgs::ColorRGBA createColor(double r, double g, double b, double a) {
        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }

    // 成员变量
    ros::NodeHandle nh_;
    ros::Subscriber model_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher trail_pub_;

    std::string global_frame_;
    std::string model_prefix_;
    std::string mesh_resource_;
    std_msgs::ColorRGBA uniform_color_;
    
    // 轨迹颜色相关
    std::vector<std_msgs::ColorRGBA> trail_colors_;
    std::unordered_map<std::string, std_msgs::ColorRGBA> color_map_;
    int color_index_;

    // 轨迹存储（无上限）
    std::unordered_map<std::string, std::vector<geometry_msgs::Point>> trail_map_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multiUAV_visualizer");
    UnlimitedDroneVisualizer visualizer;
    ros::spin();
    return 0;
}