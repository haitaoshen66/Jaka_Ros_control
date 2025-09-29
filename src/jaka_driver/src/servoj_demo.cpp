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
using namespace std;

BOOL in_pos;
JAKAZuRobot robot;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("client_test");
    auto servo_move_enable_client = node->create_client<jaka_msgs::srv::ServoMoveEnable>("/jaka_driver/servo_move_enable");
    auto servo_j_client = node->create_client<jaka_msgs::srv::ServoMove>("/jaka_driver/servo_j");

    // Waiting for service to be available
    while (!servo_move_enable_client->wait_for_service(chrono::seconds(1)) ||
           !servo_j_client->wait_for_service(chrono::seconds(1))) 
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

    auto servo_pose = make_shared<jaka_msgs::srv::ServoMove::Request>();
    float pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.01};
    for (int i =0; i < 6; i++)
    {
        servo_pose->pose.push_back(pose[i]);
    } 

    for (int i = 0; i < 157; i++)
    {
        auto future_servo_j = servo_j_client->async_send_request(servo_pose);
        if (rclcpp::spin_until_future_complete(node, future_servo_j) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future_servo_j.get();  // Wait for the result
            cout << "Servo move attempt: " << i + 1 << " - ret " << result->ret << ", " << result->message << endl;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("client_test"), "Failed to execute servo move on attempt %d", (i+1));
            return 1;
        }
    }
    
    this_thread::sleep_for(chrono::seconds(1));

    rclcpp::shutdown();
    return 0;
}
