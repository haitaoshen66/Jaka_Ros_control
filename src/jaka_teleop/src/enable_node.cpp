#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "jaka_msgs/srv/move.hpp"
#include "jaka_driver/JAKAZuRobot.h"

#include <memory>
#include <thread>
#include <chrono>

using std::placeholders::_1;
using std::shared_ptr;

JAKAZuRobot robot;

class ReEnableNode : public rclcpp::Node {
public:
    ReEnableNode() : Node("re_enable_node") {
        // 初始化机器人连接
        RCLCPP_INFO(this->get_logger(), "🔧 登录机器人并上电中...");
        if (robot.login_in("192.168.10.90") != 0 ||
            robot.power_on() != 0 ||
            robot.enable_robot() != 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ 登录 / 上电 / 使能失败");
            throw std::runtime_error("机器人初始化失败");
        }
        RCLCPP_INFO(this->get_logger(), "✅ 机器人已连接并初始上电");

        // 创建 joint_move 服务客户端
        joint_move_client_ = this->create_client<jaka_msgs::srv::Move>("/jaka_driver/joint_move");
        joint_move_client_->wait_for_service();

        // 订阅重新使能话题
        re_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/teleop/enable", 10, std::bind(&ReEnableNode::re_enable_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "✅ ReEnableNode 已启动，监听 /teleop/enable");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr re_enable_sub_;
    rclcpp::Client<jaka_msgs::srv::Move>::SharedPtr joint_move_client_;

    void re_enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;

        RCLCPP_INFO(this->get_logger(), "📥 接收到重新使能请求，开始恢复流程");

        // 再次尝试登录 + 上电 + 使能（防止异常断连）
        if (robot.login_in("192.168.10.90") != 0 ||
            robot.power_on() != 0 ||
            robot.enable_robot() != 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ 恢复流程失败（登录/上电/使能）");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ 已成功重新登录并使能");

        // 创建 joint move 请求（回归初始姿态）
        auto request = std::make_shared<jaka_msgs::srv::Move::Request>();
        request->pose = {0.0, 0.0, 1.57, -1.57, 1.57, 0.0};
        request->has_ref = false;
        request->ref_joint = {0};
        request->mvvelo = 0.5;
        request->mvacc = 0.5;
        request->mvtime = 0.0;
        request->mvradii = 0.0;
        request->coord_mode = 0;
        request->index = 0;

        auto future = joint_move_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "🏁 机器人已回归初始姿态");
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 回归初始姿态失败");
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReEnableNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
