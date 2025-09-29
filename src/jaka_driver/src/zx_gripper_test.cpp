#include "rclcpp/rclcpp.hpp"
#include "jaka_driver/JAKAZuRobot.h"
#include <vector>
#include <memory>
#include <chrono>
#include <thread>

// ZX CRC16 & Modbus组包
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

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    JAKAZuRobot robot;
    robot.login_in("192.168.10.90");
    robot.power_on();
    robot.enable_robot();
    RCLCPP_INFO(rclcpp::get_logger("zx_gripper_test"), "Robot ready");

    // ZX夹爪初始化
    std::vector<unsigned char> init_data = {0x01, 0x06, 0x01, 0x00, 0x00, 0xA5};
    std::vector<unsigned char> init_cmd = getGripperModbusCode(init_data);
    robot.send_tio_rs_command(2, init_cmd.data(), init_cmd.size());
    RCLCPP_INFO(rclcpp::get_logger("zx_gripper_test"), "ZX gripper init");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 设置夹爪力30
    int16_t force = 30;
    unsigned char force_high = (force >> 8) & 0xFF;
    unsigned char force_low = force & 0xFF;
    std::vector<unsigned char> force_data = {0x01, 0x06, 0x00, 0x0b, force_high, force_low};
    std::vector<unsigned char> force_cmd = getGripperModbusCode(force_data);
    robot.send_tio_rs_command(2, force_cmd.data(), force_cmd.size());
    RCLCPP_INFO(rclcpp::get_logger("zx_gripper_test"), "ZX gripper force set to 30");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 打开夹爪
    std::vector<unsigned char> open_data = {0x01, 0x06, 0x00, 0x29, 0x00, 0x01};
    std::vector<unsigned char> open_cmd = getGripperModbusCode(open_data);
    robot.send_tio_rs_command(2, open_cmd.data(), open_cmd.size());
    RCLCPP_INFO(rclcpp::get_logger("zx_gripper_test"), "ZX gripper open");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 关闭夹爪
    std::vector<unsigned char> close_data = {0x01, 0x06, 0x00, 0x28, 0x00, 0x01};
    std::vector<unsigned char> close_cmd = getGripperModbusCode(close_data);
    robot.send_tio_rs_command(2, close_cmd.data(), close_cmd.size());
    RCLCPP_INFO(rclcpp::get_logger("zx_gripper_test"), "ZX gripper close");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    robot.disable_robot();
    robot.power_off();
    robot.login_out();
    rclcpp::shutdown();
    return 0;
}
