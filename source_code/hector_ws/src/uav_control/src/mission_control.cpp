#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <quad_star/PathPlan.h>
#include <geometry_msgs/PoseArray.h>

class MissionController {
private:
    ros::NodeHandle nh_;  // 私有句柄（带~命名空间）
    ros::ServiceServer start_service;
    ros::ServiceClient plan_client;
    ros::Publisher path_pub;
    std::string uav_name;

    bool startCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
        ROS_INFO("%s: Starting mission...", uav_name.c_str());

        // 1. 准备路径规划请求（通过私有句柄获取参数）
        quad_star::PathPlan plan_srv;
        nh_.getParam("start_x", plan_srv.request.start_x);
        nh_.getParam("start_y", plan_srv.request.start_y);
        nh_.getParam("goal_x", plan_srv.request.goal_x);
        nh_.getParam("goal_y", plan_srv.request.goal_y);

        // 设置默认值
        if(!nh_.hasParam("start_x")) plan_srv.request.start_x = -9.0;
        if(!nh_.hasParam("start_y")) plan_srv.request.start_y = -9.0;
        if(!nh_.hasParam("goal_x")) plan_srv.request.goal_x = 9.0;
        if(!nh_.hasParam("goal_y")) plan_srv.request.goal_y = 9.0;

        // 2. 调用路径规划服务
        if(plan_client.call(plan_srv)) {
            if(plan_srv.response.success) {
                // 3. 发布路径到跟踪器
                geometry_msgs::PoseArray path_msg;
                path_msg.header.stamp = ros::Time::now();
                path_msg.header.frame_id = "world";

                for(const auto& point : plan_srv.response.path) {
                    geometry_msgs::Pose pose;
                    pose.position = point;
                    path_msg.poses.push_back(pose);
                }

                path_pub.publish(path_msg);
                ROS_INFO("%s: Published path with %zu waypoints", 
                        uav_name.c_str(), path_msg.poses.size());
                return true;
            } else {
                ROS_ERROR("%s: Planning failed: %s", 
                        uav_name.c_str(), plan_srv.response.message.c_str());
            }
        } else {
            ROS_ERROR("%s: Failed to call planning service", uav_name.c_str());
        }
        return false;
    }

public:
    MissionController() : nh_("~") {  // 初始化私有句柄
        // 获取无人机名称参数
        nh_.param<std::string>("uav_name", uav_name, "uav1");

        // 初始化服务（注意话题/服务名称需包含uav_name前缀）
        start_service = nh_.advertiseService(
            "start_mission",  // 自动添加私有前缀 -> /uav1/mission_control/start_mission
            &MissionController::startCallback, this);
            
        // 服务客户端（需确保路径规划服务在全局命名空间）
        plan_client = nh_.serviceClient<quad_star::PathPlan>(
            "/path_planning_service");  // 硬编码全局路径
            
        // 发布器（自动添加私有前缀）
        path_pub = nh_.advertise<geometry_msgs::PoseArray>(
            "path", 1);  // 实际发布到 /uav1/mission_control/path

        ROS_INFO("%s mission controller ready. Call service %s/start_mission to begin.",
               uav_name.c_str(), nh_.resolveName("start_mission").c_str());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_controller");
    MissionController controller;
    ros::spin();
    return 0;
}