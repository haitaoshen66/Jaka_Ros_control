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
#include "jaka_msgs/srv/get_ik.hpp"
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
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

using namespace std;

BOOL in_pos;
JAKAZuRobot robot;

// 从CSV文件读取位姿数据
std::vector<std::vector<float>> readPosesFromCSV(const std::string &filename)
{
    std::vector<std::vector<float>> poses;
    std::ifstream file(filename);

    if (!file.is_open())
    {
        throw std::runtime_error("无法打开文件: " + filename);
    }

    std::string line;
    bool isHeader = true; // 用于跳过表头行

    while (std::getline(file, line))
    {
        if (isHeader)
        {
            isHeader = false;
            continue; // 跳过表头行
        }

        std::stringstream ss(line);
        std::vector<float> pose;
        std::string value;
        int column = 0;

        while (std::getline(ss, value, ','))
        {
            column++;
            if (column == 1)
                continue; // 跳过frame列

            try
            {
                pose.push_back(std::stof(value));
            }
            catch (const std::exception &e)
            {
                std::cerr << "解析错误: " << value << " - " << e.what() << std::endl;
            }
        }

        if (pose.size() == 6)
        { // roll,pitch,yaw,x,y,z - 6个值
            poses.push_back(pose);
        }
        else
        {
            std::cerr << "警告: 跳过无效行，应有6个值，实际有 " << pose.size() << " 个值" << std::endl;
        }
    }

    return poses;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("servoj_increment_demo");

    // 创建所需的客户端
    auto get_ik_client = node->create_client<jaka_msgs::srv::GetIK>("/jaka_driver/get_ik");
    auto servo_move_enable_client = node->create_client<jaka_msgs::srv::ServoMoveEnable>("/jaka_driver/servo_move_enable");
    auto servo_j_client = node->create_client<jaka_msgs::srv::ServoMove>("/jaka_driver/servo_j");

    // 定义CSV文件路径
    std::string csv_file_path = "/home/dija/DIJA/questVR_ws/src/oculus_reader/scripts/right_hand_xyz_delta.csv";
    if (argc > 1)
    {
        csv_file_path = argv[1];
    }

    RCLCPP_INFO(node->get_logger(), "使用CSV文件: %s", csv_file_path.c_str());

    // 读取CSV文件中的位姿数据
    std::vector<std::vector<float>> poses;
    try
    {
        poses = readPosesFromCSV(csv_file_path);
        RCLCPP_INFO(node->get_logger(), "成功读取 %lu 个位姿数据", poses.size());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "读取CSV文件失败: %s", e.what());
        return 1;
    }

    if (poses.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "CSV文件中没有有效的位姿数据");
        return 1;
    }

    // 等待服务可用
    while (!get_ik_client->wait_for_service(chrono::seconds(1)) ||
           !servo_move_enable_client->wait_for_service(chrono::seconds(1)) ||
           !servo_j_client->wait_for_service(chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "等待服务时被中断。退出。");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "等待服务可用...");
    }

    // 启用servo模式
    RCLCPP_INFO(node->get_logger(), "服务已找到！启用servo模式...");
    auto enable_request = std::make_shared<jaka_msgs::srv::ServoMoveEnable::Request>();
    enable_request->enable = true;
    auto future_enable = servo_move_enable_client->async_send_request(enable_request);

    if (rclcpp::spin_until_future_complete(node, future_enable) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Servo模式已启用！");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "启用Servo模式失败");
        return 1;
    }

    this_thread::sleep_for(chrono::seconds(1));

    // 初始关节角度，作为IK参考
    std::vector<float> current_joint_position = {0.0, 0.0, 1.57, -1.57, 1.57, 0.0};
    std::vector<float> prev_joint_position = current_joint_position;

    // 初始位置（与ros2_auto.py中相同）
    std::vector<float> init_position = {154, -396.7, 211, -0.785, -1.57, -2.36};

    // 处理每个姿态点
    double start_time = rclcpp::Clock().now().seconds();
    auto servo_request = std::make_shared<jaka_msgs::srv::ServoMove::Request>();
    for (size_t i = 0; i < poses.size(); i++)
    {
        // 计算绝对笛卡尔位置
        std::vector<float> cartesian_pose = {
            poses[i][3] + init_position[0], // x
            poses[i][4] + init_position[1], // y
            poses[i][5] + init_position[2], // z
            poses[i][0] + init_position[3], // roll
            poses[i][1] + init_position[4], // pitch
            poses[i][2] + init_position[5]  // yaw
        };
        init_position[0] += poses[i][3];
        init_position[1] += poses[i][4];
        init_position[2] += poses[i][5];
        init_position[3] += poses[i][0];
        init_position[4] += poses[i][1];
        init_position[5] += poses[i][2];
        // 调用GetIK服务
        auto ik_request = std::make_shared<jaka_msgs::srv::GetIK::Request>();
        ik_request->ref_joint = current_joint_position;
        ik_request->cartesian_pose = cartesian_pose;

        RCLCPP_INFO(node->get_logger(), "调用GetIK服务，笛卡尔位姿: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    cartesian_pose[0], cartesian_pose[1], cartesian_pose[2],
                    cartesian_pose[3], cartesian_pose[4], cartesian_pose[5]);

        auto future_ik = get_ik_client->async_send_request(ik_request);
        if (rclcpp::spin_until_future_complete(node, future_ik) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "获取IK解决方案失败");
            continue;
        }

        auto ik_result = future_ik.get();
        if (ik_result->joint[0] == 9999.0)
        {
            RCLCPP_ERROR(node->get_logger(), "IK求解失败: %s", ik_result->message.c_str());
            continue;
        }

        std::vector<float> new_joint_position(ik_result->joint.begin(), ik_result->joint.end());
        RCLCPP_INFO(node->get_logger(), "获取IK解决方案成功: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    new_joint_position[0], new_joint_position[1], new_joint_position[2],
                    new_joint_position[3], new_joint_position[4], new_joint_position[5]);

        // 计算关节增量
        std::vector<float> joint_increments(6);
        for (int j = 0; j < 6; j++)
        {
            joint_increments[j] = new_joint_position[j] - current_joint_position[j];
        }

        RCLCPP_INFO(node->get_logger(), "关节增量: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    joint_increments[0], joint_increments[1], joint_increments[2],
                    joint_increments[3], joint_increments[4], joint_increments[5]);

        // 使用servo_j发送关节增量

        servo_request->pose = joint_increments;

        auto future_servo = servo_j_client->async_send_request(servo_request);
        if (rclcpp::spin_until_future_complete(node, future_servo) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto servo_result = future_servo.get();
            RCLCPP_INFO(node->get_logger(), "Servo移动结果: %d - %s",
                        servo_result->ret, servo_result->message.c_str());

            // 更新当前关节位置作为下次IK的参考
            prev_joint_position = current_joint_position;
            current_joint_position = new_joint_position;

            // 添加短暂延时保证平滑运动
            // this_thread::sleep_for(chrono::milliseconds(50));
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "执行Servo移动失败");
        }
    }

    double end_time = rclcpp::Clock().now().seconds();
    RCLCPP_INFO(node->get_logger(), "完成所有位姿移动，总耗时: %.2f 秒", end_time - start_time);

    // 禁用servo模式
    enable_request->enable = false;
    future_enable = servo_move_enable_client->async_send_request(enable_request);
    if (rclcpp::spin_until_future_complete(node, future_enable) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "已禁用Servo模式");
    }

    rclcpp::shutdown();
    return 0;
}