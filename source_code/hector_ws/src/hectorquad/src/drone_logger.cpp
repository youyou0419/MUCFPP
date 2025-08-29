#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fstream>
#include <map>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
#include <ctime>

// 无人机数据记录器
class MultiDroneLogger {
public:
    explicit MultiDroneLogger(ros::NodeHandle& nh) : nh_(nh) {
        init();
    }

    ~MultiDroneLogger() {
        closeAllFiles();
    }

private:
    struct DroneData {
        std::ofstream csv_file;
        std::string name;
    };

    ros::NodeHandle nh_;
    std::map<std::string, DroneData> drones_;
    std::vector<ros::Subscriber> odom_subs_;
    std::vector<ros::Subscriber> imu_subs_;
    ros::Subscriber model_states_sub_;
    std::ofstream moving_box_file_;
    std::string output_dir_;

    void init() {
        // 获取输出目录参数
        nh_.param<std::string>("output_dir", output_dir_, getDefaultOutputDir());
        
        // 确保输出目录存在
        if (!ensureDirectoryExists(output_dir_)) {
            ROS_ERROR_STREAM("Failed to create output directory: " << output_dir_);
            ros::shutdown();
            return;
        }

        // 初始化无人机记录器
        std::vector<std::string> drone_names = {"uav1", "uav2", "uav3", "uav4"};
        for (const auto& name : drone_names) {
            if (!initDroneLogger(name)) {
                ROS_ERROR_STREAM("Failed to initialize logger for drone: " << name);
            }
        }

        // 初始化moving_box记录器
        if (!initMovingBoxLogger()) {
            ROS_ERROR("Failed to initialize moving_box logger");
        }

        // 如果没有任何无人机初始化成功，则关闭节点
        if (drones_.empty()) {
            ROS_ERROR("No drone loggers initialized successfully");
            ros::shutdown();
        }
    } 

    std::string getDefaultOutputDir() const {
        // 默认输出到桌面目录下的drone_logs文件夹
        std::string home_path = getenv("HOME");
        return home_path + "/hector_ws/drone_logs/proposed/dynamic";
    }

    bool ensureDirectoryExists(const std::string& path) {
        struct stat info;
        if (stat(path.c_str(), &info) != 0) {
            // 目录不存在，尝试递归创建
            std::string cmd = "mkdir -p " + path;
            if (system(cmd.c_str()) != 0) {
                ROS_ERROR_STREAM("Failed to create directory: " << path);
                return false;
            }
            ROS_INFO_STREAM("Created directory: " << path);
        } else if (!(info.st_mode & S_IFDIR)) {
            ROS_ERROR_STREAM("Path exists but is not a directory: " << path);
            return false;
        }
        return true;
    }

    bool initDroneLogger(const std::string& drone_name) {
        // 创建带时间戳的子目录
        std::time_t now = std::time(nullptr);
        std::tm* now_tm = std::localtime(&now);
        char time_buf[20];
        std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d", now_tm);
        std::string daily_dir = output_dir_ + "/" + time_buf;
        
        if (!ensureDirectoryExists(daily_dir)) {
            return false;
        }

        // 初始化数据结构
        drones_[drone_name].name = drone_name;
        std::string file_path = daily_dir + "/" + drone_name + "_log.csv";
        
        // 检查文件是否已存在
        bool file_exists = std::ifstream(file_path).good();

        // 打开文件（追加模式）
        drones_[drone_name].csv_file.open(file_path, std::ios::out | std::ios::app);
        
        if (!drones_[drone_name].csv_file.is_open()) {
            ROS_ERROR_STREAM("Failed to open CSV file for " << drone_name << " at " << file_path);
            return false;
        }
        
        // 如果是新文件，写入表头
        if (!file_exists) {
            drones_[drone_name].csv_file << "timestamp,x,y,z,vx,vy,vz,roll,pitch,yaw\n";
            drones_[drone_name].csv_file.flush();
        }
        
        ROS_INFO_STREAM("Recording " << drone_name << " state to " << file_path);

        // 创建订阅者
        odom_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
            "/" + drone_name + "/ground_truth/state", 10,
            [this, drone_name](const nav_msgs::Odometry::ConstPtr& msg) {
                odomCallback(msg, drone_name);
            }));

        imu_subs_.push_back(nh_.subscribe<sensor_msgs::Imu>(
            "/" + drone_name + "/raw_imu", 10,
            [this, drone_name](const sensor_msgs::Imu::ConstPtr& msg) {
                imuCallback(msg, drone_name);
            }));

        return true;
    }

    bool initMovingBoxLogger() {
        // 创建带时间戳的子目录
        std::time_t now = std::time(nullptr);
        std::tm* now_tm = std::localtime(&now);
        char time_buf[20];
        std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d", now_tm);
        std::string daily_dir = output_dir_ + "/" + time_buf;
        
        if (!ensureDirectoryExists(daily_dir)) {
            return false;
        }

        std::string file_path = daily_dir + "/moving_box_log.csv";
        
        // 检查文件是否已存在
        bool file_exists = std::ifstream(file_path).good();

        // 打开文件（追加模式）
        moving_box_file_.open(file_path, std::ios::out | std::ios::app);
        
        if (!moving_box_file_.is_open()) {
            ROS_ERROR_STREAM("Failed to open CSV file for moving_box at " << file_path);
            return false;
        }
        
        // 如果是新文件，写入表头
        if (!file_exists) {
            moving_box_file_ << "timestamp,x,y,z,vx,vy,vz,roll,pitch,yaw\n";
            moving_box_file_.flush();
        }
        
        ROS_INFO_STREAM("Recording moving_box state to " << file_path);

        // 订阅model_states话题
        model_states_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>(
            "/gazebo/model_states", 10,
            [this](const gazebo_msgs::ModelStates::ConstPtr& msg) {
                modelStatesCallback(msg);
            });

        return true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& drone_name) {
        auto it = drones_.find(drone_name);
        if (it == drones_.end() || !it->second.csv_file.is_open()) {
            ROS_WARN_STREAM("Received data for uninitialized drone: " << drone_name);
            return;
        }

        double timestamp = msg->header.stamp.toSec();

        // 位置
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;

        // 速度
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double vz = msg->twist.twist.linear.z;

        // 姿态（四元数转欧拉角）
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 写入CSV
        it->second.csv_file << timestamp << ","
                          << x << "," << y << "," << z << ","
                          << vx << "," << vy << "," << vz << ","
                          << roll << "," << pitch << "," << yaw << "\n";
        
        // 定期刷新缓冲区
        if (++write_count_ % 10 == 0) {
            it->second.csv_file.flush();
        }
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        if (!moving_box_file_.is_open()) {
            return;
        }

        // 查找moving_box在模型列表中的索引
        auto it = std::find(msg->name.begin(), msg->name.end(), "moving_box");
        if (it == msg->name.end()) {
            return; // moving_box不在模型列表中
        }

        size_t index = std::distance(msg->name.begin(), it);
        if (index >= msg->pose.size() || index >= msg->twist.size()) {
            return; // 索引超出范围
        }

        double timestamp = ros::Time::now().toSec();

        // 位置
        double x = msg->pose[index].position.x;
        double y = msg->pose[index].position.y;
        double z = msg->pose[index].position.z;

        // 速度
        double vx = msg->twist[index].linear.x;
        double vy = msg->twist[index].linear.y;
        double vz = msg->twist[index].linear.z;

        // 姿态（四元数转欧拉角）
        tf2::Quaternion q(
            msg->pose[index].orientation.x,
            msg->pose[index].orientation.y,
            msg->pose[index].orientation.z,
            msg->pose[index].orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 写入CSV
        moving_box_file_ << timestamp << ","
                        << x << "," << y << "," << z << ","
                        << vx << "," << vy << "," << vz << ","
                        << roll << "," << pitch << "," << yaw << "\n";
        
        // 定期刷新缓冲区
        if (++moving_box_write_count_ % 10 == 0) {
            moving_box_file_.flush();
        }
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg, const std::string& drone_name) {
        // 如果需要记录IMU数据，可以在这里实现
        // 当前主要记录Odometry数据
    }

    void closeAllFiles() {
        for (auto& drone : drones_) {
            if (drone.second.csv_file.is_open()) {
                drone.second.csv_file.close();
                ROS_INFO_STREAM("Closed log file for " << drone.first);
            }
        }
        
        if (moving_box_file_.is_open()) {
            moving_box_file_.close();
            ROS_INFO("Closed log file for moving_box");
        }
    }

    int write_count_ = 0;
    int moving_box_write_count_ = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_drone_logger");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    MultiDroneLogger logger(nh);
    ros::Rate rate(10);
    while (ros::ok()) {
        // 获取并记录数据
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Multi-drone logger is running...");
    ros::spin();

    return 0;
}