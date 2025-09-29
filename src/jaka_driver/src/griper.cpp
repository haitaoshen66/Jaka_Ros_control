#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_msgs/srv/servo_move_enable.hpp"
#include<thread>
#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
using namespace std;

// -------------------- DH Gripper (原始代码备份) --------------------
#if 0
#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_msgs/srv/servo_move_enable.hpp"
#include<thread>
#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
using namespace std;
JAKAZuRobot robot;  
uint16_t calculateModbusCRC(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF; // 初始值
    for (size_t i = 0; i < length; i++) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}
bool gripper_init_enable_callback(const shared_ptr<jaka_msgs::srv::ServoMoveEnable::Request> request,
    shared_ptr<jaka_msgs::srv::ServoMoveEnable::Response> response)
{
    uint8_t init_cmd[] = {0x01,0x06,0x01,0x00,0x00,0xA5,0x48,0x4D};
    robot.send_tio_rs_command(2, init_cmd, sizeof(init_cmd));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gripper init");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    uint8_t force_cmd[] = {0x01, 0x06, 0x01, 0x01, 0x00, 0x1E, 0x59, 0xFE};
    robot.send_tio_rs_command(2, force_cmd, sizeof(force_cmd));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gripper power set to 30");
    response->message = "Gripper initialized and power set to 30%";
    return true;
}
bool gripper_set_callback(const shared_ptr<jaka_msgs::srv::ServoMoveEnable::Request> request,
    shared_ptr<jaka_msgs::srv::ServoMoveEnable::Response> response)
{
    bool open = request->enable;
    if (open) {
        uint8_t open_cmd[] = {0x01, 0x06, 0x01, 0x03, 0x03, 0xE8, 0x78, 0x88};
        robot.send_tio_rs_command(2, open_cmd, sizeof(open_cmd));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gripper open");
        response->ret = 1;
        response->message = "Gripper opened";
    }
    else{
        uint8_t pos_cmd[] = {0x01, 0x06, 01, 03, 0x0, 0x04, 0x78, 0x21};
        uint16_t crc = calculateModbusCRC(pos_cmd, sizeof(pos_cmd) - 2);
        pos_cmd[6] = crc & 0xFF;
        pos_cmd[7] = (crc >> 8) & 0xFF;
        robot.send_tio_rs_command(2, pos_cmd, sizeof(pos_cmd));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gripper close");
        response->ret = 0;
        response->message = "Gripper closed";
    }
    return true;
}
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    robot.login_in("192.168.10.90");  
    robot.power_on();
    robot.enable_robot();
}
#endif

// --------------------    ZX Gripper    --------------------
#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_msgs/srv/servo_move_enable.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
using namespace std;
JAKAZuRobot robot;
std::pair<unsigned char, unsigned char> CRC16(const std::vector<unsigned char>& data) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < data.size(); i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return {static_cast<unsigned char>(crc & 0xFF), static_cast<unsigned char>((crc >> 8) & 0xFF)};
}
std::vector<unsigned char> getGripperModbusCode(const std::vector<unsigned char>& data){
    if(data.size() != 6){
        throw std::invalid_argument("Input data must be exactly 6 bytes.");
    }
    auto [crc_low, crc_high] = CRC16(data);
    std::vector<unsigned char> modbus_data = data;
    modbus_data.push_back(crc_low);
    modbus_data.push_back(crc_high);
    return modbus_data;
}
bool gripper_init_enable_callback(const shared_ptr<jaka_msgs::srv::ServoMoveEnable::Request> request,
    shared_ptr<jaka_msgs::srv::ServoMoveEnable::Response> response)
{
    std::vector<unsigned char> base_data = {0x01, 0x06, 0x01, 0x00, 0x00, 0xA5};
    std::vector<unsigned char> init_cmd = getGripperModbusCode(base_data);
    robot.send_tio_rs_command(2, init_cmd.data(), init_cmd.size());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ZX gripper init");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    int16_t force = 30;
    unsigned char force_high = (force >> 8) & 0xFF;
    unsigned char force_low = force & 0xFF;
    std::vector<unsigned char> force_data = {0x01, 0x06, 0x00, 0x0b, force_high, force_low};
    std::vector<unsigned char> force_cmd = getGripperModbusCode(force_data);
    robot.send_tio_rs_command(2, force_cmd.data(), force_cmd.size());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ZX gripper force set to 30");
    response->message = "ZX Gripper initialized and force set to 30";
    return true;
}
bool gripper_set_callback(const shared_ptr<jaka_msgs::srv::ServoMoveEnable::Request> request,
    shared_ptr<jaka_msgs::srv::ServoMoveEnable::Response> response)
{
    bool open = request->enable;
    if (open) {
        std::vector<unsigned char> open_data = {0x01, 0x06, 0x00, 0x29, 0x00, 0x01};
        std::vector<unsigned char> open_cmd = getGripperModbusCode(open_data);
        robot.send_tio_rs_command(2, open_cmd.data(), open_cmd.size());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ZX gripper open");
        response->ret = 1;
        response->message = "ZX Gripper opened";
    }
    else{
        std::vector<unsigned char> close_data = {0x01, 0x06, 0x00, 0x28, 0x00, 0x01};
        std::vector<unsigned char> close_cmd = getGripperModbusCode(close_data);
        robot.send_tio_rs_command(2, close_cmd.data(), close_cmd.size());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ZX gripper close");
        response->ret = 0;
        response->message = "ZX Gripper closed";
    }
    return true;
}
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    robot.login_in("192.168.10.90");  
    robot.power_on();
    robot.enable_robot();
}
// ZX夹爪控制实现
     // Ensure thread is joined before shutting down the node

    // rclcpp::shutdown();
    // robot.login_out();   


// ZX夹爪控制实现
