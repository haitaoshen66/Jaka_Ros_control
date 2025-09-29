#include "gripper_control/zx_gripper_ctl.hpp"

namespace hardware_ctl {

    std::vector<unsigned char> getGripperModbusCode(const std::vector<unsigned char>& data){
        if(data.size() != 6){
            throw std::invalid_argument("Input data must be exactly 6 bytes.");
        }
        // 计算CRC
        auto [crc_low, crc_high] = CRC16(data);
        std::vector<unsigned char> modbus_data = data;
        modbus_data.push_back(crc_low);
        modbus_data.push_back(crc_high);
        return modbus_data;
    }

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

    void init_gripper(std::shared_ptr<xarm_api::XArmROSClient> xarm_client){
        // 设置timeout
        // int gripper_timeout = 600;
        // 设置波特率
        int gripper_baudrate = 115200;

        // 初始化modbus data
        // std::vector<unsigned char> send_modbus_data = {0x01, 0x06, 0x01, 0x00, 0x00, 0xA5, 0x48, 0x4D};
        // int send_data_length = send_modbus_data.size();
        // std::vector<unsigned char> recv_modbus_data;
        // int recv_data_length = 8;

        // xarm_client->set_tgpio_modbus_timeout(gripper_timeout);
        xarm_client->set_tgpio_modbus_baudrate(gripper_baudrate);
        // xarm_client->getset_tgpio_modbus_data(send_modbus_data, send_data_length, recv_modbus_data, recv_data_length);
        // set_gripper_force(xarm_client, 30);
        RCLCPP_INFO(rclcpp::get_logger("dh_gripper_ctl"), "Gripper initialized.");
    }

    void open_gripper(std::shared_ptr<xarm_api::XArmROSClient> xarm_client){
        // 打开夹爪modbus data
        std::vector<unsigned char> send_modbus_data = {0x01, 0x06, 0x00, 0x29, 0x00, 0x01};
        int send_data_length = send_modbus_data.size();
        std::vector<unsigned char> recv_modbus_data;
        int recv_data_length = 8;

        xarm_client->getset_tgpio_modbus_data(send_modbus_data, send_data_length, recv_modbus_data, recv_data_length);
        RCLCPP_INFO(rclcpp::get_logger("dh_gripper_ctl"), "Gripper opened.");
    }

    void close_gripper(std::shared_ptr<xarm_api::XArmROSClient> xarm_client){
        // 关闭夹爪modbus data
        std::vector<unsigned char> send_modbus_data = {0x01, 0x06, 0x00, 0x28, 0x00, 0x01};
        int send_data_length = send_modbus_data.size();
        std::vector<unsigned char> recv_modbus_data;
        int recv_data_length = 8;

        xarm_client->getset_tgpio_modbus_data(send_modbus_data, send_data_length, recv_modbus_data, recv_data_length);
        // printf("recv_modbus_data[7]: %d\n", recv_modbus_data[7]);
        RCLCPP_INFO(rclcpp::get_logger("dh_gripper_ctl"), "Gripper closed.");
    }

    int16_t get_gripper_force(std::shared_ptr<xarm_api::XArmROSClient> xarm_client){
        // 获取夹爪力度modbus data
        std::vector<unsigned char> send_modbus_data = {0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0xD4, 0x36};
        int send_data_length = send_modbus_data.size();
        std::vector<unsigned char> recv_modbus_data;
        int recv_data_length = 7;

        xarm_client->getset_tgpio_modbus_data(send_modbus_data, send_data_length, recv_modbus_data, recv_data_length);
        RCLCPP_INFO(rclcpp::get_logger("dh_gripper_ctl"), "Gripper force obtained.");
        return ((recv_modbus_data[3] << 8) | recv_modbus_data[4]);
    }

    void set_gripper_force(std::shared_ptr<xarm_api::XArmROSClient> xarm_client, int16_t force){
        if(force < 20 || force > 320){
            throw std::out_of_range("Gripper force must be between 20 and 320");
        }

        // 设置夹爪力度modbus data
        unsigned char force_high = (force >> 8) & 0xFF;
        unsigned char force_low = force & 0xFF;
        std::vector<unsigned char> base_data = {0x01, 0x06, 0x00, 0x0b, force_high, force_low};
        // std::vector<unsigned char> send_modbus_data = getGripperModbusCode(base_data);
        int send_data_length = base_data.size();
        std::vector<unsigned char> recv_modbus_data;
        int recv_data_length = 8;
        
        xarm_client->getset_tgpio_modbus_data(base_data, send_data_length, recv_modbus_data, recv_data_length);
        RCLCPP_INFO(rclcpp::get_logger("dh_gripper_ctl"), "Gripper force set successfully.");
    }
    
    int16_t get_gripper_position(std::shared_ptr<xarm_api::XArmROSClient> xarm_client){
        // 获取夹爪位置modbus data
        std::vector<unsigned char> send_modbus_data = {0x01, 0x03, 0x00, 0x0a, 0x00, 0x01};
        int send_data_length = send_modbus_data.size();
        std::vector<unsigned char> recv_modbus_data;
        int recv_data_length = 7;

        xarm_client->getset_tgpio_modbus_data(send_modbus_data, send_data_length, recv_modbus_data, recv_data_length);
        RCLCPP_INFO(rclcpp::get_logger("dh_gripper_ctl"), "Gripper position obtained.");
        return ((recv_modbus_data[3] << 8) | recv_modbus_data[4]);
    }

    void set_gripper_position(std::shared_ptr<xarm_api::XArmROSClient> xarm_client, int16_t position){
        if(position < 1 || position > 100){
            throw std::out_of_range("Position must be between 1 and 100");
        }

        // 设置夹爪位置modbus data
        unsigned char pos_high = (position >> 8) & 0xFF;
        unsigned char pos_low = position & 0xFF;
        std::vector<unsigned char> base_data = {0x01, 0x06, 0x00, 0x0a, pos_high, pos_low};
        // std::vector<unsigned char> send_modbus_data = getGripperModbusCode(base_data);
        int send_data_length = base_data.size();
        std::vector<unsigned char> recv_modbus_data;
        int recv_data_length = 8;
        xarm_client->getset_tgpio_modbus_data(base_data, send_data_length, recv_modbus_data, recv_data_length);
        RCLCPP_INFO(rclcpp::get_logger("dh_gripper_ctl"), "Gripper position set successfully.");
    }

    void sys(std::shared_ptr<xarm_api::XArmROSClient> xarm_client){
        // 关闭夹爪modbus data
        std::vector<unsigned char> send_modbus_data = {0x01, 0x06, 0x00, 0x2a, 0x00, 0x01};
        int send_data_length = send_modbus_data.size();
        std::vector<unsigned char> recv_modbus_data;
        int recv_data_length = 8;

        xarm_client->getset_tgpio_modbus_data(send_modbus_data, send_data_length, recv_modbus_data, recv_data_length);
        // printf("recv_modbus_data[7]: %d\n", recv_modbus_data[7]);
        RCLCPP_INFO(rclcpp::get_logger("dh_gripper_ctl"), "Sys successfully!.");
    }
} // namespace hardware_ctl
