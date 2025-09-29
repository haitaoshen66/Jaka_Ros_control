#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

#include "jaka_msgs/msg/robot_msg.hpp"
#include "jaka_msgs/srv/move.hpp"
#include "jaka_msgs/srv/servo_move_enable.hpp"
#include "jaka_msgs/srv/servo_move.hpp"
#include "jaka_msgs/srv/set_user_frame.hpp"
#include "jaka_msgs/srv/set_tcp_frame.hpp"
#include "jaka_msgs/srv/set_payload.hpp"
#include "jaka_msgs/srv/set_collision.hpp"
#include "jaka_msgs/srv/clear_error.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"

#include <string>
#include <fstream>  // 用于文件操作
#include <sstream>  // 用于字符串流处理
#include <vector>   // 用于存储多行数据

using namespace std;

BOOL in_pos;
JAKAZuRobot robot;

// 从CSV文件读取位姿数据
std::vector<std::vector<float>> readPosesFromCSV(const std::string& filename) {
    std::vector<std::vector<float>> poses;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件: " + filename);
    }
    
    std::string line;
    bool isHeader = true;  // 用于跳过表头行
    
    while (std::getline(file, line)) {
        if (isHeader) {
            isHeader = false;
            continue;  // 跳过表头行
        }
        
        std::stringstream ss(line);
        std::vector<float> pose;
        std::string value;
        int column = 0;
        
        while (std::getline(ss, value, ',')) {
            column++;
            if (column == 1) continue;  // 跳过frame列
            
            try {
                pose.push_back(std::stof(value));
            } catch (const std::exception& e) {
                std::cerr << "解析错误: " << value << " - " << e.what() << std::endl;
            }
        }
        
        if (pose.size() == 6) {  // roll,pitch,yaw,x,y,z - 6个值
            // 重新排列为x,y,z,roll,pitch,yaw顺序
            std::vector<float> reordered = {
                pose[3], pose[4], pose[5],  // x, y, z
                pose[0], pose[1], pose[2]   // roll, pitch, yaw
            };
            poses.push_back(reordered);
        } else {
            std::cerr << "警告: 跳过无效行，应有6个值，实际有 " << pose.size() << " 个值" << std::endl;
        }
    }
    
    return poses;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("client_test");
    auto servo_move_enable_client = node->create_client<jaka_msgs::srv::ServoMoveEnable>("/jaka_driver/servo_move_enable");
    auto servo_p_client = node->create_client<jaka_msgs::srv::ServoMove>("/jaka_driver/servo_p");

    // 定义CSV文件路径
    std::string csv_file_path = "/home/dija/DIJA/questVR_ws/src/oculus_reader/scripts/right_hand_xyz_delta.csv";  // 默认文件名
    if (argc > 1) {
        csv_file_path = argv[1];  // 如果提供了命令行参数，使用参数作为CSV文件路径
    }
    
    RCLCPP_INFO(rclcpp::get_logger("client_test"), "使用CSV文件: %s", csv_file_path.c_str());
    
    // 读取CSV文件中的位姿数据
    std::vector<std::vector<float>> poses;
    try {
        poses = readPosesFromCSV(csv_file_path);
        RCLCPP_INFO(rclcpp::get_logger("client_test"), "成功读取 %lu 个位姿数据", poses.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("client_test"), "读取CSV文件失败: %s", e.what());
        return 1;
    }
    
    if (poses.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("client_test"), "CSV文件中没有有效的位姿数据");
        return 1;
    }

    // Waiting for service to be available
    while (!servo_move_enable_client->wait_for_service(chrono::seconds(1)) ||
           !servo_p_client->wait_for_service(chrono::seconds(1))) 
    {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("client_test"), "Interrupted while waiting for the service. Exiting.");
            return 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("client_test"), "Waiting for service...");
    }
    RCLCPP_INFO(rclcpp::get_logger("client_test"), "Services found! Enabling servo mode..." );

    auto enable_state = make_shared<jaka_msgs::srv::ServoMoveEnable::Request>();
    enable_state->enable = true;
    auto future_enable = servo_move_enable_client->async_send_request(enable_state);

    if (rclcpp::spin_until_future_complete(node, future_enable) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("client_test"), "Servo mode enabled!");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("client_test"), "Failed to enable servo mode");
        return 1;
    }

    this_thread::sleep_for(chrono::seconds(1));
    
    double start = rclcpp::Clock().now().seconds();
    
    // 对CSV中的每一行位姿数据执行servo_move
    for (size_t i = 0; i < poses.size(); i++)
    {
        auto servo_pose = make_shared<jaka_msgs::srv::ServoMove::Request>();
        
        // 使用CSV文件中的数据
        for (int j = 0; j < 6; j++) {
            servo_pose->pose.push_back(poses[i][j]);
        }
        
        auto future_servo_p = servo_p_client->async_send_request(servo_pose);
        if (rclcpp::spin_until_future_complete(node, future_servo_p) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future_servo_p.get();
            cout << "位姿 " << i + 1 << ": [" 
                 << poses[i][0] << ", " << poses[i][1] << ", " << poses[i][2] << ", "
                 << poses[i][3] << ", " << poses[i][4] << ", " << poses[i][5] << "] - ret "
                 << result->ret << ", " << result->message << endl;
            
            // 添加适当的延时，确保机器人能够平滑移动
            this_thread::sleep_for(chrono::milliseconds(500));
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("client_test"), "执行第 %ld 个位姿失败", (i+1));
        }
    }
    
    double end = rclcpp::Clock().now().seconds();
    double duration = end - start;
    RCLCPP_INFO(rclcpp::get_logger("client_test"), "执行 %lu 个位姿移动总耗时: %.2f 秒", poses.size(), duration);
    this_thread::sleep_for(chrono::seconds(1));

    rclcpp::shutdown();
    return 0;
}