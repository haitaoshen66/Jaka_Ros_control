#include <cstdlib>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "jaka_msgs/msg/robot_msg.hpp"
#include "jaka_msgs/srv/move.hpp"
#include "jaka_msgs/srv/servo_move_enable.hpp"
#include "jaka_msgs/srv/servo_move.hpp"
#include "jaka_msgs/srv/set_user_frame.hpp"
#include "jaka_msgs/srv/set_tcp_frame.hpp"
#include "jaka_msgs/srv/set_payload.hpp"
#include "jaka_msgs/srv/set_collision.hpp"
#include "jaka_msgs/srv/set_io.hpp"
#include "jaka_msgs/srv/get_io.hpp"
#include "jaka_msgs/srv/clear_error.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"

#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <string>
#include <map>
#include <chrono>
#include <thread>
using namespace std;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Get two addends from terminal
    if (argc != 7)
    {
        RCLCPP_INFO(rclcpp::get_logger("linear_move_client"), "bbq0");
        return 1;
    }

    auto node = rclcpp::Node::make_shared("linear_move_client");
    auto client = node->create_client<jaka_msgs::srv::Move>("/jaka_driver/linear_move");
    auto request = make_shared<jaka_msgs::srv::Move::Request>();

    RCLCPP_INFO(rclcpp::get_logger("linear_move_client"), "bbq3");
    
    for (int i = 0; i <6; i++)
    {
        request->pose.push_back(atof(argv[i+1]));
    }
    // srv.request.pose[0] = -376.0;
    // ROS_INFO("bbq4");
    // srv.request.pose[1] = atof(argv[2]);
    // ROS_INFO("bbq5");
    // srv.request.pose[2] = atof(argv[3]);
    // srv.request.pose[3] = atof(argv[4]);
    // srv.request.pose[4] = atof(argv[5]);
    // srv.request.pose[5] = atof(argv[6]);
    request->mvvelo = 100;
    request->mvacc = 100;
	// srv.request.coord_mode=0;
	// srv.request.index=0;

    // Wait for the service to be available
    while (!client->wait_for_service(chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("linear_move_client"), "Interrupted while waiting for the service. Exiting.");
            return 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("linear_move_client"), "Waiting for service...");
    }

    // Send the service request asynchronously
    auto future = client->async_send_request(request);

    // Handle the response
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("linear_move_client"), "SUCCESS!");
        RCLCPP_INFO(rclcpp::get_logger("linear_move_client"), "ret: %d", (int)future.get()->ret);
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("linear_move_client"), "bbq6");
        RCLCPP_ERROR(rclcpp::get_logger("linear_move_client"), "Failed to call service");
        return 1;
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}