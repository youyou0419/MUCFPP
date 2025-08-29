#include <ros/ros.h>
#include <uav_control/DeadlockInfo.h>
#include <uav_control/ResolutionStatus.h>
#include <uav_control/LDRCompletion.h>  // 新增：LDR完成消息
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/MarkerArray.h>
#include <uav_control/LDRInfo.h>
#include <uav_control/LDRInfoArray.h>
#include <unordered_map>
#include <set>
#include <algorithm>

class DeadlockResolver {
private:
    ros::NodeHandle nh_;
    ros::Subscriber deadlock_sub_;
    ros::Subscriber model_states_sub_;
    ros::Subscriber resolution_status_sub_;
    ros::Subscriber ldr_completion_sub_;  // 新增：LDR完成消息订阅者
    ros::Publisher ldr_pub_;
    ros::Publisher aabb_viz_pub_;
    ros::Publisher ldr_info_pub_;

    // 无人机状态结构
    struct UAVStatus {
        bool in_deadlock = false;
        bool in_resolution = false;
        std::set<std::string> related_drones;
        geometry_msgs::Point position;
    };

    // 存储所有无人机状态
    std::unordered_map<std::string, UAVStatus> uav_status_;

    // AABB包围盒结构
    struct AABB {
        std::set<std::string> uavs;
        double min_x, max_x;
        double min_y, max_y;
    };

    std::vector<AABB> last_ldr_instances_;          // 上一次检测到的LDR实例
    std::vector<AABB> last_published_ldr_instances_;  // 上一次发布的LDR实例
    bool first_update_ = true;

public:
    DeadlockResolver() {
        deadlock_sub_ = nh_.subscribe("/deadlock_info", 10, &DeadlockResolver::deadlockCallback, this);
        model_states_sub_ = nh_.subscribe("/gazebo/model_states", 1, &DeadlockResolver::modelStatesCallback, this);
        resolution_status_sub_ = nh_.subscribe("/resolution_status", 10, &DeadlockResolver::resolutionStatusCallback, this);
        
        // 新增：订阅LDR完成消息
        ldr_completion_sub_ = nh_.subscribe("/ldr_completion", 10, &DeadlockResolver::ldrCompletionCallback, this);
        
        ldr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ldr_instances", 1);
        aabb_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/aabb_visualization", 1);
        ldr_info_pub_ = nh_.advertise<uav_control::LDRInfoArray>("/ldr_info", 1, true); // latch=true保证订阅者能收到最新消息
        
        ROS_INFO("DeadlockResolver initialized with LDR completion monitoring");
    }

    // 新增：LDR完成消息回调函数
    void ldrCompletionCallback(const uav_control::LDRCompletion::ConstPtr& msg) {
        ROS_INFO("Received LDR completion message for LDR %d", msg->ldr_id);
        
        // 将相应的无人机的in_deadlock状态设置为false
        for (const auto& uav_id : msg->uav_ids) {
            if (uav_status_.find(uav_id) != uav_status_.end()) {
                bool was_in_deadlock = uav_status_[uav_id].in_deadlock;
                
                // 清除deadlock状态和相关无人机信息
                uav_status_[uav_id].in_deadlock = false;
                uav_status_[uav_id].related_drones.clear();
                
                if (was_in_deadlock) {
                    ROS_INFO("UAV %s deadlock status cleared due to LDR %d completion", 
                             uav_id.c_str(), msg->ldr_id);
                } else {
                    ROS_DEBUG("UAV %s was not in deadlock when LDR %d completed", 
                              uav_id.c_str(), msg->ldr_id);
                }
            } else {
                ROS_WARN("Unknown UAV %s in LDR %d completion message", 
                         uav_id.c_str(), msg->ldr_id);
            }
        }
        
        // 输出完成信息
        ROS_INFO("LDR completion message: %s", msg->message.c_str());
        
        // 由于deadlock状态发生变化，触发一次LDR实例重新计算
        // 这将在下一次modelStatesCallback中自动处理
        ROS_DEBUG("LDR completion processed, will trigger LDR recalculation on next update");
    }

    void resolutionStatusCallback(const uav_control::ResolutionStatus::ConstPtr& msg) {
        if (msg->in_resolution) {
            // 将相应ID的无人机设置为in_resolution状态
            for (const auto& uav_id : msg->uav_ids) {
                uav_status_[uav_id].in_resolution = true;
                ROS_INFO("UAV %s entered resolution phase for LDR %d", uav_id.c_str(), msg->ldr_id);
            }
        } else {
            // 退出resolution状态
            for (const auto& uav_id : msg->uav_ids) {
                uav_status_[uav_id].in_resolution = false;
                ROS_INFO("UAV %s exited resolution phase for LDR %d", uav_id.c_str(), msg->ldr_id);
            }
        }
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        // 更新无人机位置
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string& name = msg->name[i];
            if (name.find("uav") == 0) {
                uav_status_[name].position = msg->pose[i].position;
            }
        }
        processDeadlocks();
    }

    void deadlockCallback(const uav_control::DeadlockInfo::ConstPtr& msg) {
        if (msg->is_bilateral) {
            uav_status_[msg->blocker_id].in_deadlock = true;
            uav_status_[msg->blocker_id].related_drones.insert(msg->blocked_id);
            uav_status_[msg->blocked_id].in_deadlock = true;
            uav_status_[msg->blocked_id].related_drones.insert(msg->blocker_id);
            ROS_DEBUG("Bilateral deadlock detected: %s <-> %s", 
                      msg->blocker_id.c_str(), msg->blocked_id.c_str());
        } else {
            uav_status_[msg->blocked_id].in_deadlock = true;
            uav_status_[msg->blocked_id].related_drones.insert(msg->blocker_id);
            ROS_DEBUG("Unilateral deadlock detected: %s blocked by %s", 
                      msg->blocked_id.c_str(), msg->blocker_id.c_str());
        }
    }

    // 比较两个LDR实例集合是否完全相同
    bool areLDRInstancesSame(const std::vector<AABB>& a, const std::vector<AABB>& b) {
        if (a.size() != b.size()) return false;
        for (const auto& box_a : a) {
            bool found = false;
            for (const auto& box_b : b) {
                if (box_a.uavs == box_b.uavs && 
                    std::abs(box_a.min_x - box_b.min_x) < 0.1 &&
                    std::abs(box_a.max_x - box_b.max_x) < 0.1 &&
                    std::abs(box_a.min_y - box_b.min_y) < 0.1 &&
                    std::abs(box_a.max_y - box_b.max_y) < 0.1) {
                    found = true;
                    break;
                }
            }
            if (!found) return false;
        }
        return true;
    }

    void processDeadlocks() {
        std::vector<AABB> aabb_list;
        
        // 1. 生成初始AABB（排除处于resolution状态的无人机）
        for (const auto& pair : uav_status_) {
            if (pair.second.in_deadlock && !pair.second.in_resolution) {
                AABB box;
                box.uavs.insert(pair.first);
                box.min_x = pair.second.position.x - 2.0;
                box.max_x = pair.second.position.x + 2.0;
                box.min_y = pair.second.position.y - 2.0;
                box.max_y = pair.second.position.y + 2.0;
                aabb_list.push_back(box);
            }
        }
        
        // 2. 合并AABB
        bool changed;
        do {
            changed = false;
            for (size_t i = 0; i < aabb_list.size(); ) {
                bool merged = false;
                for (size_t j = i + 1; j < aabb_list.size(); ) {
                    if (checkAABBIntersection(aabb_list[i], aabb_list[j])) {
                        mergeAABBs(aabb_list[i], aabb_list[j]);
                        aabb_list.erase(aabb_list.begin() + j);
                        changed = true;
                        merged = true;
                    } else {
                        j++;
                    }
                }
                if (merged) i = 0;
                else i++;
            }
        } while (changed);
        
        // 3. 扩展相关无人机（但排除处于resolution状态的）
        for (auto& box : aabb_list) {
            std::set<std::string> to_add;
            for (const auto& uav : box.uavs) {
                for (const auto& related : uav_status_[uav].related_drones) {
                    if (box.uavs.count(related) == 0 && 
                        uav_status_[related].in_deadlock && 
                        !uav_status_[related].in_resolution) {
                        to_add.insert(related);
                    }
                }
            }
            for (const auto& uav : to_add) {
                box.uavs.insert(uav);
                expandAABB(box, uav_status_[uav].position);
            }
        }
        
        // 4. 筛选有效LDR实例
        std::vector<AABB> valid_ldr_instances;
        for (const auto& box : aabb_list) {
            bool valid = true;
            for (const auto& pair : uav_status_) {
                if (!pair.second.in_deadlock && isPointInAABB(pair.second.position, box)) {
                    valid = false;
                    break;
                }
            }
            if (valid && box.uavs.size() >= 2) {
                valid_ldr_instances.push_back(box);
            }
        }
        
        // 5. 检查稳定性
        bool ldr_stable = false;
        if (first_update_) {
            first_update_ = false;
        } else {
            ldr_stable = areLDRInstancesSame(last_ldr_instances_, valid_ldr_instances);
        }
        
        // 6. 发布逻辑
        if (ldr_stable) {
            // 只有当前实例与上一次发布不同时才会发布
            if (!areLDRInstancesSame(last_published_ldr_instances_, valid_ldr_instances)) {
                publishLDRInfo(valid_ldr_instances);
            }
        }
        
        // 7. 更新记录
        last_ldr_instances_ = valid_ldr_instances;
        
        // 8. 可视化（始终更新）
        // publishLDRInstances(valid_ldr_instances);
        // visualizeAABBs(valid_ldr_instances);
    }

    void publishLDRInfo(const std::vector<AABB>& instances) {
        uav_control::LDRInfoArray ldr_info_array;
        for (const auto& instance : instances) {
            uav_control::LDRInfo ldr_info;
            ldr_info.min_x = instance.min_x;
            ldr_info.max_x = instance.max_x;
            ldr_info.min_y = instance.min_y;
            ldr_info.max_y = instance.max_y;
            for (const auto& uav_id : instance.uavs) {
                ldr_info.uav_ids.push_back(uav_id);
                ldr_info.positions.push_back(uav_status_[uav_id].position);
            }
            ldr_info_array.ldr_infos.push_back(ldr_info);
        }
        ldr_info_pub_.publish(ldr_info_array);
        last_published_ldr_instances_ = instances; // 记录本次发布内容
        ROS_INFO("Published updated LDR info with %zu instances", instances.size());
    }

    bool checkAABBIntersection(const AABB& a, const AABB& b) {
        return !(a.max_x < b.min_x || a.min_x > b.max_x || 
                a.max_y < b.min_y || a.min_y > b.max_y);
    }

    void mergeAABBs(AABB& a, const AABB& b) {
        a.uavs.insert(b.uavs.begin(), b.uavs.end());
        a.min_x = std::min(a.min_x, b.min_x);
        a.max_x = std::max(a.max_x, b.max_x);
        a.min_y = std::min(a.min_y, b.min_y);
        a.max_y = std::max(a.max_y, b.max_y);
    }

    void expandAABB(AABB& box, const geometry_msgs::Point& pos) {
        box.min_x = std::min(box.min_x, pos.x - 2.0);
        box.max_x = std::max(box.max_x, pos.x + 2.0);
        box.min_y = std::min(box.min_y, pos.y - 2.0);
        box.max_y = std::max(box.max_y, pos.y + 2.0);
    }

    bool isPointInAABB(const geometry_msgs::Point& p, const AABB& box) {
        return p.x >= box.min_x && p.x <= box.max_x &&
               p.y >= box.min_y && p.y <= box.max_y;
    }

    void publishLDRInstances(const std::vector<AABB>& instances) {
        visualization_msgs::MarkerArray marker_array;
        int id = 0;
        
        for (const auto& instance : instances) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "ldr_instances";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            
            // 计算中心点和尺寸
            marker.pose.position.x = (instance.min_x + instance.max_x) / 2;
            marker.pose.position.y = (instance.min_y + instance.max_y) / 2;
            marker.pose.position.z = 1.0; // 适当高度
            
            marker.scale.x = instance.max_x - instance.min_x;
            marker.scale.y = instance.max_y - instance.min_y;
            marker.scale.z = 0.1; // 薄层
            
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5; // 半透明
            
            marker.lifetime = ros::Duration(0.5); // 短暂显示
            
            marker_array.markers.push_back(marker);
            
            // 添加文本标记显示无人机ID
            visualization_msgs::Marker text_marker;
            text_marker.header = marker.header;
            text_marker.ns = "ldr_text";
            text_marker.id = id++;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position = marker.pose.position;
            text_marker.pose.position.z += 1.0;
            
            std::string text = "LDR: ";
            for (const auto& uav : instance.uavs) {
                text += uav + " ";
            }
            text_marker.text = text;
            
            text_marker.scale.z = 0.5; // 文字大小
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            marker_array.markers.push_back(text_marker);
        }
        
        ldr_pub_.publish(marker_array);
    }

    void visualizeAABBs(const std::vector<AABB>& instances) {
        visualization_msgs::MarkerArray marker_array;
        int id = 0;
        
        for (const auto& instance : instances) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "aabb_boxes";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            
            // 设置AABB边框
            marker.scale.x = 0.1; // 线宽
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            
            // 绘制矩形边框
            geometry_msgs::Point p;
            p.z = 0.5; // 适当高度
            
            p.x = instance.min_x; p.y = instance.min_y;
            marker.points.push_back(p);
            
            p.x = instance.max_x; p.y = instance.min_y;
            marker.points.push_back(p);
            
            p.x = instance.max_x; p.y = instance.max_y;
            marker.points.push_back(p);
            
            p.x = instance.min_x; p.y = instance.max_y;
            marker.points.push_back(p);
            
            p.x = instance.min_x; p.y = instance.min_y;
            marker.points.push_back(p);
            
            marker_array.markers.push_back(marker);
        }
        
        aabb_viz_pub_.publish(marker_array);
    }

    // 新增：获取当前系统状态的调试函数
    void printSystemStatus() {
        ROS_INFO("=== DeadlockResolver System Status ===");
        int deadlock_count = 0;
        int resolution_count = 0;
        
        for (const auto& pair : uav_status_) {
            if (pair.second.in_deadlock) {
                deadlock_count++;
                ROS_INFO("UAV %s: in_deadlock=true, in_resolution=%s, related_drones=%zu", 
                         pair.first.c_str(), 
                         pair.second.in_resolution ? "true" : "false",
                         pair.second.related_drones.size());
            }
            if (pair.second.in_resolution) {
                resolution_count++;
            }
        }
        
        ROS_INFO("Total UAVs in deadlock: %d", deadlock_count);
        ROS_INFO("Total UAVs in resolution: %d", resolution_count);
        ROS_INFO("LDR instances detected: %zu", last_ldr_instances_.size());
        ROS_INFO("======================================");
    }

    void run() {
        ros::Rate rate(10); // 10Hz处理频率
        
        // 可选：定期打印系统状态
        ros::Time last_status_print = ros::Time::now();
        double status_print_interval = 30.0; // 每30秒打印一次状态
        
        while (ros::ok()) {
            ros::spinOnce();
            
            // 定期打印系统状态（调试用）
            if ((ros::Time::now() - last_status_print).toSec() > status_print_interval) {
                printSystemStatus();
                last_status_print = ros::Time::now();
            }
            
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "deadlock_resolver");
    ROS_INFO("Starting DeadlockResolver with LDR completion monitoring...");
    
    DeadlockResolver resolver;
    resolver.run();
    
    ROS_INFO("DeadlockResolver shutting down");
    return 0;
}