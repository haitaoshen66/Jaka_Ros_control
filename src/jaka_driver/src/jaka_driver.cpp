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
#include "jaka_msgs/srv/set_io.hpp"
#include "jaka_msgs/srv/get_io.hpp"
#include "jaka_msgs/srv/get_fk.hpp"
#include "jaka_msgs/srv/get_ik.hpp"
#include "jaka_msgs/srv/clear_error.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"
#include "std_msgs/msg/bool.hpp"

#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>


#include <string>
#include <map>
#include <chrono>
#include <thread>
using namespace std;

const double PI = 3.1415926;
//Define variable: the direction that was sent down the last time the jog was called
int jog_index_last = -1; 
//Define variable: number of calls to jog
int jog_count = 0;
//Define variable: save the number of jog calls
int jog_count_temp = 0;
JAKAZuRobot robot;
//SDK interface return status
map<int, string>mapErr = {
    {2,"ERR_FUCTION_CALL_ERROR"},
    {-1,"ERR_INVALID_HANDLER"},
    {-2,"ERR_INVALID_PARAMETER"},
    {-3,"ERR_COMMUNICATION_ERR"},
    {-4,"ERR_KINE_INVERSE_ERR"},
    {-5,"ERR_EMERGENCY_PRESSED"},
    {-6,"ERR_NOT_POWERED"},
    {-7,"ERR_NOT_ENABLED"},
    {-8,"ERR_DISABLE_SERVOMODE"},
    {-9,"ERR_NOT_OFF_ENABLE"},
    {-10,"ERR_PROGRAM_IS_RUNNING"},
    {-11,"ERR_CANNOT_OPEN_FILE"},
    {-12,"ERR_MOTION_ABNORMAL"}
};

// Declare publishers
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr tool_position_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_position_pub;
rclcpp::Publisher<jaka_msgs::msg::RobotMsg>::SharedPtr robot_state_pub;

rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr re_enable_sub;
rclcpp::Client<jaka_msgs::srv::Move>::SharedPtr joint_move_client;

std::atomic<bool> is_reset_running(false);
std::chrono::steady_clock::time_point last_reset_time = std::chrono::steady_clock::now() - std::chrono::seconds(5);

uint16_t calculateModbusCRC(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF; // 初始值
    
    for (size_t i = 0; i < length; i++) {
        crc ^= static_cast<uint16_t>(data[i]); // 将当前字节与CRC寄存器异或
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001; // 如果最低位为1，右移并异或多项式
            } else {
                crc = crc >> 1; // 否则只右移
            }
        }
    }
    return crc;
}
bool gripper_init_enable_callback(const shared_ptr<jaka_msgs::srv::ServoMoveEnable::Request> request,
    shared_ptr<jaka_msgs::srv::ServoMoveEnable::Response> response)
{
    // int ret = robot.servo_move_enable(enable);
    uint8_t init_cmd[] = {0x01,0x06,0x01,0x00,0x00,0xA5,0x48,0x4D};
    robot.send_tio_rs_command(2, init_cmd, sizeof(init_cmd));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gripper init");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // 设置力值30%
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
        uint8_t open_cmd[] = {0x01, 0x06, 0x01, 0x03, 0x03, 0xE8, 0x78, 0x88}; // 位置1000
        robot.send_tio_rs_command(2, open_cmd, sizeof(open_cmd));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gripper open");
        response->ret = 1;
        response->message = "Gripper opened";
    }
    else{
        uint8_t pos_cmd[] = {0x01, 0x06, 01, 03, 0x0, 0x04, 0x78, 0x21}; //pose 0
        // for(int i = 0; i < 10; i++) {
        uint16_t crc = calculateModbusCRC(pos_cmd, sizeof(pos_cmd) - 2);
        pos_cmd[6] = crc & 0xFF; // CRC低字节
        pos_cmd[7] = (crc >> 8) & 0xFF;
        robot.send_tio_rs_command(2, pos_cmd, sizeof(pos_cmd));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gripper close");
        response->ret = 0;
        response->message = "Gripper closed";
    }
    return true;
}
bool linear_move_callback(const shared_ptr<jaka_msgs::srv::Move::Request> request,
    shared_ptr<jaka_msgs::srv::Move::Response> response)
{
    CartesianPose end_pose;
    double speed = static_cast<double>(request->mvvelo);
    double accel = static_cast<double>(request->mvacc);
    double tol = 0.5;
    // Rpy rpy;
    OptionalCond *option_cond = nullptr;
    end_pose.tran.x = request->pose[0];
    end_pose.tran.y = request->pose[1];
    end_pose.tran.z = request->pose[2];
    Eigen::Vector3d Angaxis = {request->pose[3], request->pose[4], request->pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(end_pose.rpy));
    
    // Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
    // auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
    // end_pose.rpy.rx = rpy.x();
    // end_pose.rpy.ry = rpy.y();
    // end_pose.rpy.rz = rpy.z();
    
    int ret = robot.linear_move(&end_pose, MoveMode::ABS, TRUE, speed, accel, tol, option_cond);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "linear_move has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }

    return true;

}

bool joint_move_callback(const shared_ptr<jaka_msgs::srv::Move::Request> request,
    shared_ptr<jaka_msgs::srv::Move::Response> response)
{
    JointValue joint_pose;
    joint_pose.jVal[0] = request->pose[0];
    joint_pose.jVal[1] = request->pose[1];
    joint_pose.jVal[2] = request->pose[2];
    joint_pose.jVal[3] = request->pose[3];
    joint_pose.jVal[4] = request->pose[4]; 
    joint_pose.jVal[5] = request->pose[5];
    double speed = static_cast<double>(request->mvvelo);
    double accel = static_cast<double>(request->mvacc);
    double tol = 0.5;
    OptionalCond *option_cond = nullptr;

    int ret = robot.joint_move(&joint_pose, MoveMode::ABS, true, speed, accel, tol, option_cond);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "joint_move has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool jog_callback(const shared_ptr<jaka_msgs::srv::Move::Request> request,
    shared_ptr<jaka_msgs::srv::Move::Response> response)
{
    // 1. Initialization parameters
    double move_velocity = 0;
    CoordType coord_type = COORD_JOINT;
    
    // 2. Select index   mapping the index and velocity
    //request.index 如果是关节空间   就是  0+,0-,1+,1-,2+,2-,3+,3-,4+,4-,5+,5-
    //request.index 如果是笛卡尔空间 就是x+,x-,y+,y-,z+,z-,rx+,rx-,ry+,ry-,rz+,rz-
    float index_temp = static_cast<float>(request->index) / 2 + 0.1;
    int index = static_cast<int>(index_temp);
    
    // 3. Select coordinates
    switch (request->coord_mode)
    {
        case 0:
            //coordinate system of joints
            coord_type = COORD_JOINT; 
            //Joint Movement Velocity (rad/s)
            move_velocity = request->mvacc;       
            break;
        case 1:
            //Base coordinate system (Cartesian space)
            coord_type = COORD_BASE;
            //movement speed (mm/s)
            move_velocity = request->mvacc;  
            break;
        case 2:
            //Tool coordinate system (Cartesian space)
		    coord_type = COORD_TOOL;
            //movement speed (mm/s)
            move_velocity = request->mvacc;
            break; 
        default:
            RCLCPP_INFO(rclcpp::get_logger("jog_callback"), "Coordinate system input error, please re-enter");
            return true;
    }
    // 4. Determine the direction of velocity 
    //Determine whether robot motion (articulated or Cartesian) is in a positive or negative direction
    if(request->index & 1)
    {
        move_velocity = -move_velocity;
    }
    //5. Conducting jogging
    if (jog_index_last != request->index)
    {   
        int ret = robot.motion_abort();
        if (ret == 0)
        {
            int jog_state = robot.jog(index, CONTINUE, coord_type, move_velocity, 0);
            switch(jog_state)
            {
                case 0:
                    response->ret = 1;
                    response->message = "Position is reached";
                    break;
                default:
                    response->ret = jog_state;
                    response->message = "error occurred:" + mapErr[jog_state];
                    break;
            }
        }
        else
        {
            response->ret = ret;
            response->message = "error occurred:" + mapErr[ret];
        }
        jog_index_last = request->index;
    }
    else
    {
        response->ret = 1;
        response->message = "Robot is jogging";
        RCLCPP_INFO(rclcpp::get_logger("jog_callback"), "Robot is jogging");
    }
    jog_count = jog_count + 1;
    return true;
}

bool servo_move_enable_callback(const shared_ptr<jaka_msgs::srv::ServoMoveEnable::Request> request,
    shared_ptr<jaka_msgs::srv::ServoMoveEnable::Response> response)
{
    BOOL enable = request->enable;
    int ret = robot.servo_move_enable(enable);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "servo_move_enable has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool servo_p_callback(const shared_ptr<jaka_msgs::srv::ServoMove::Request> request,
    shared_ptr<jaka_msgs::srv::ServoMove::Response> response)
{
    //speed * 0.008
    CartesianPose cartesian_pose;
    cartesian_pose.tran.x = request->pose[0];
    cartesian_pose.tran.y = request->pose[1];
    cartesian_pose.tran.z = request->pose[2];
    cartesian_pose.rpy.rx = request->pose[3];
    cartesian_pose.rpy.ry = request->pose[4];
    cartesian_pose.rpy.rz = request->pose[5];
    int ret = robot.servo_p(&cartesian_pose, MoveMode::INCR);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "Servo_p has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool servo_j_callback(const shared_ptr<jaka_msgs::srv::ServoMove::Request> request,
    shared_ptr<jaka_msgs::srv::ServoMove::Response> response)
{
    JointValue joint_pose;
    joint_pose.jVal[0] = request->pose[0];
    joint_pose.jVal[1] = request->pose[1];
    joint_pose.jVal[2] = request->pose[2];
    joint_pose.jVal[3] = request->pose[3];
    joint_pose.jVal[4] = request->pose[4];
    joint_pose.jVal[5] = request->pose[5];
    int ret = robot.servo_j(&joint_pose, MoveMode::INCR);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "Servo_j has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool stop_move_callback([[maybe_unused]] const shared_ptr<std_srvs::srv::Empty::Request> request,
    [[maybe_unused]] shared_ptr<std_srvs::srv::Empty::Response> response)
{
    //Initialize jog related parameters
    jog_count = 0;
    jog_count_temp = 0;
    jog_index_last = -1;
    int ret = robot.motion_abort();
    switch(ret)
    {
        case 0:
            RCLCPP_INFO(rclcpp::get_logger("stop_move_callback"), "stop_move has been executed");
            break;
        default:
            RCLCPP_INFO(rclcpp::get_logger("stop_move_callback"), "error occurred: %s", mapErr[ret].c_str());
            return false;;
    }
    return true;
}


bool set_toolFrame_callback(const shared_ptr<jaka_msgs::srv::SetTcpFrame::Request> request,
    shared_ptr<jaka_msgs::srv::SetTcpFrame::Response> response)
{
    CartesianPose tool_frame;
    int tool_frame_id = request->tool_num;
    tool_frame.tran.x = request->pose[0];
    tool_frame.tran.y = request->pose[1];
    tool_frame.tran.z = request->pose[2];
    Eigen::Vector3d Angaxis = {request->pose[3],request->pose[4],request->pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(tool_frame.rpy));
    // Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
    // auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
    // tool_frame.rpy.rx = rpy.x();
    // tool_frame.rpy.ry = rpy.y();
    // tool_frame.rpy.rz = rpy.z();

    int ret = robot.set_tool_data(tool_frame_id, &tool_frame, "ToolCoord");
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "set_toolFrame has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}


bool set_userFrame_callback(const shared_ptr<jaka_msgs::srv::SetUserFrame::Request> request,
    shared_ptr<jaka_msgs::srv::SetUserFrame::Response> response)
{
    CartesianPose user_frame;
    int user_frame_id = request->user_num; 
    user_frame.tran.x = request->pose[0];
    user_frame.tran.y = request->pose[1];
    user_frame.tran.z = request->pose[2];
    Eigen::Vector3d Angaxis = {request->pose[3],request->pose[4],request->pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(user_frame.rpy));
    // Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
    // auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
    // user_frame.rpy.rx = rpy.x();
    // user_frame.rpy.ry = rpy.y();
    // user_frame.rpy.rz = rpy.z();
    int ret = robot.set_user_frame_data(user_frame_id, &user_frame, "BaseCoord");
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "set_userFrame has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool set_payload_callback(const shared_ptr<jaka_msgs::srv::SetPayload::Request> request,
    shared_ptr<jaka_msgs::srv::SetPayload::Response> response)
{
    PayLoad payload;
    int tool_id = request->tool_num;

    payload.centroid.x = request->xc;
    payload.centroid.y = request->yc;
    payload.centroid.z = request->zc;
    payload.mass = request->mass;

    robot.set_tool_id(tool_id);
    int ret = robot.set_payload(&payload);

    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "set_payload has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }

    return true;
}

bool drag_mode_callback(const shared_ptr<std_srvs::srv::SetBool::Request> request,
    shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    int ret = robot.drag_mode_enable(request->data);
    switch(ret)
    {
        case 0:
            response->success = 1;
            response->message = "drag_mode has been executed";
            break;
        default:
            response->success = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }

    return true;

}

bool set_collisionLevel_callback(const shared_ptr<jaka_msgs::srv::SetCollision::Request> request,
    shared_ptr<jaka_msgs::srv::SetCollision::Response> response)
{
    int collision_level;
    if(request->is_enable == 0)
    {
        collision_level = 0;
    }
    else
    {
        if(request->value <= 25)
        {
            collision_level = 1;
        }
        else if(request->value <= 50)
        {
            collision_level = 2;
        }
        else if(request->value <= 75)
        {
            collision_level = 3;
        }
        else if(request->value <=100)
        {
            collision_level = 4;
        }
        else
        {
            collision_level = 5;
        }
    }
    int ret = robot.set_collision_level(collision_level);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "Collision level" + to_string(collision_level) + " has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;

    }
    return true;
}


bool set_io_callback(const shared_ptr<jaka_msgs::srv::SetIO::Request> request,
    shared_ptr<jaka_msgs::srv::SetIO::Response> response)
{   
    IOType type;
    int ret;
    switch(request->type)
    {
        case 0:
            type = IO_CABINET;
            break;
        case 1:
            type = IO_TOOL;
            break;
        case 2:
            type = IO_EXTEND;
            break;
    }
    float value = request->value;
    string signal = request->signal;
    int index = request->index;
    if(signal == "digital")
    {      
        BOOL digital_value;
        if(value)
        {
            digital_value = TRUE;
        }
        else
        {
            digital_value = FALSE;
        }
        ret = robot.set_digital_output(type, index, digital_value);
    }
    else if(signal == "analog")
    {
        ret = robot.set_analog_output(type, index, value);
    }
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "set IO has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}



bool get_io_callback(const shared_ptr<jaka_msgs::srv::GetIO::Request> request,
    shared_ptr<jaka_msgs::srv::GetIO::Response> response)
{   
    IOType type;
    int ret;
    BOOL digital_result;
    float analog_result;
    switch(request->type)
    {
        case 0:
            type = IO_CABINET;
            break;
        case 1:
            type = IO_TOOL;
            break;
        case 2:
            type = IO_EXTEND;
            break;
        default:
            response->value = -999999;
            response->message = "Invalid IO type";
            return false;  // Add return if an invalid type is requested
    }
    string signal = request->signal;
    int index = request->index;
    int path = request->path;
    if(signal == "digital")
    {       
        if(path == 0)
        {
            ret = robot.get_digital_input(type, index, &digital_result);
        }
        else if(path == 1)
        {
            ret = robot.get_digital_output(type, index, &digital_result);
        }
        else
        {
            response->value = -999999;
            response->message = "Invalid path value";
            return false;  // Ensure return for invalid path
        }
        switch(ret)
        {
            case 0:
                response->value = float(digital_result);
                response->message = "get IO has been executed";
                break;
            default:
                response->value = -999999;
                response->message = "error occurred:" + mapErr[ret];
        }
        return true;
    }
    else if(signal == "analog")
    {
        if(path == 0)
        {
            ret = robot.get_analog_input(type, index, &analog_result);
        }
        else if(path == 1)
        {
            ret = robot.get_analog_output(type, index, &analog_result);
        }
        else
        {
            response->value = -999999;
            response->message = "Invalid path value";
            return false;  // Ensure return for invalid path
        }
        switch(ret)
        {
            case 0:
                response->value = analog_result;
                response->message = "get IO has been executed";
                break;
            default:
                response->value = -999999;
                response->message = "error occurred:" + mapErr[ret];

        }
    return true;
    }
    else 
    {
        // Handle case where signal is neither "digital" nor "analog"
        response->value = -999999;
        response->message = "Invalid signal type";
        return false;  // Return false if invalid signal
    }
    // This part is redundant but is here to prevent reaching the end without a return
    return true;
    
}

bool get_fk_callback(const shared_ptr<jaka_msgs::srv::GetFK::Request> request,
    shared_ptr<jaka_msgs::srv::GetFK::Response> response)
{
    JointValue joint_pose;
    CartesianPose cartesian_pose;
    for(int i = 0; i < 6; i++)
    {
        joint_pose.jVal[i] = request->joint[i];
    }
    int ret = robot.kine_forward(&joint_pose, &cartesian_pose);
    switch(ret)
    {
        case 0:
            response->cartesian_pose.push_back(cartesian_pose.tran.x);
            response->cartesian_pose.push_back(cartesian_pose.tran.y);
            response->cartesian_pose.push_back(cartesian_pose.tran.z);
            response->cartesian_pose.push_back(cartesian_pose.rpy.rx);
            response->cartesian_pose.push_back(cartesian_pose.rpy.ry);
            response->cartesian_pose.push_back(cartesian_pose.rpy.rz);
            response->message = "get FK has been executed";
            break;
        default:
            float pose_init[6] = {9999.0, 9999.0, 9999.0, 9999.0, 9999.0, 9999.0};
            for(int i = 0; i < 6; i++)
            {
                response->cartesian_pose.push_back(pose_init[i]);
            }
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;

}

bool get_ik_callback(const shared_ptr<jaka_msgs::srv::GetIK::Request> request,
    shared_ptr<jaka_msgs::srv::GetIK::Response> response)
{
    JointValue joint_pose;
    JointValue ref_joint;
    CartesianPose cartesian_pose;
    for(int i = 0; i < 6; i++)
    {
        ref_joint.jVal[i] = request->ref_joint[i];

    }
    cartesian_pose.tran.x = request->cartesian_pose[0];
    cartesian_pose.tran.y = request->cartesian_pose[1];
    cartesian_pose.tran.z = request->cartesian_pose[2];
    cartesian_pose.rpy.rx = request->cartesian_pose[3];
    cartesian_pose.rpy.ry = request->cartesian_pose[4];
    cartesian_pose.rpy.rz = request->cartesian_pose[5];
    int ret = robot.kine_inverse(&ref_joint, &cartesian_pose, &joint_pose);
    switch(ret)
    {
        case 0:
            for(int i = 0; i < 6; i++)
            {
                response->joint.push_back(joint_pose.jVal[i]);
            }
            response->message = "get IK has been executed";
            break;
        default:
            float joint_init[6] = {9999.0, 9999.0, 9999.0, 9999.0, 9999.0, 9999.0};
            for(int i = 0; i < 6; i++)
            {
                response->joint.push_back(joint_init[i]);
            }
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;

}


/*
bool clear_error_callback(const shared_ptr<jaka_msgs::srv::ClearError::Request> request,
    shared_ptr<jaka_msgs::srv::ClearError::Response> response)
{

    return true;
}
*/

void tool_position_callback(const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& tool_position_pub)
{
    // Check if publisher is valid
    if (!tool_position_pub)
    {
        RCLCPP_ERROR(rclcpp::get_logger("jaka_driver"), "Publisher is not initialized!");
        return;
    }

    geometry_msgs::msg::TwistStamped  tool_position;
    // RobotStatus robotstatus;
    CartesianPose tcp_position;
    RotMatrix rot;
    Rpy rpy;
    // robot.get_robot_status(&robotstatus);
    robot.get_tcp_position(&tcp_position);

    // tool_position.twist.linear.x = robotstatus.cartesiantran_position[0];
    // tool_position.twist.linear.y = robotstatus.cartesiantran_position[1];
    // tool_position.twist.linear.z = robotstatus.cartesiantran_position[2];
    // rpy.rx = robotstatus.cartesiantran_position[3];
    // rpy.ry = robotstatus.cartesiantran_position[4];
    // rpy.rz = robotstatus.cartesiantran_position[5];

    tool_position.twist.linear.x = tcp_position.tran.x;
    tool_position.twist.linear.y = tcp_position.tran.y;
    tool_position.twist.linear.z = tcp_position.tran.z;
    rpy.rx = tcp_position.rpy.rx;
    rpy.ry = tcp_position.rpy.ry;
    rpy.rz = tcp_position.rpy.rz;

    robot.rpy_to_rot_matrix(&rpy, &rot);
    // Eigen::Vector3d angaxis = Rot2Angaxis(rot);
    // tool_position.twist.angular.x = angaxis[0];
    // tool_position.twist.angular.y = angaxis[1];
    // tool_position.twist.angular.z = angaxis[2];
    tool_position.twist.angular.x = (rpy.rx )/PI*180;
    tool_position.twist.angular.y = (rpy.ry )/PI*180;
    tool_position.twist.angular.z = (rpy.rz )/PI*180;

    // Eigen::Vector3d eulerAngle(rpy.rx,rpy.ry,rpy.rz);
    // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
    // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));
    // Eigen::AngleAxisd rotation_vector;
    // rotation_vector=yawAngle*pitchAngle*rollAngle;
    // cout << "angle is: " << rotation_vector.angle() << endl;
    // cout << "axis is: " << rotation_vector.axis() << endl;
    // double angle = rotation_vector.angle();
    // Eigen::Vector3d axis = rotation_vector.axis();
    // Eigen::Vector3d v = angle * axis;
    // tool_position.twist.angular.x = v[0];
    // tool_position.twist.angular.y = v[1];
    // tool_position.twist.angular.z = v[2];
    
    tool_position.header.stamp = rclcpp::Clock().now();
    tool_position_pub->publish(tool_position);
}

void joint_position_callback(const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& joint_position_pub)
{
    sensor_msgs::msg::JointState joint_position;
    // RobotStatus robotstatus;
    JointValue joint_pos;
    // robot.get_robot_status(&robotstatus);
    robot.get_joint_position(&joint_pos);
    
    for (int i = 0; i < 6; i++)
    {
        // joint_position.position.push_back(robotstatus.joint_position[i]);
        // int j = i + 1;
        // joint_position.name.push_back("joint_" + to_string(j));

        joint_position.position.push_back(joint_pos.jVal[i]); 
        joint_position.name.push_back("joint_" + to_string(i + 1));
    }
    joint_position.header.stamp = rclcpp::Clock().now();
    joint_position_pub->publish(joint_position);
}

void robot_states_callback(const rclcpp::Publisher<jaka_msgs::msg::RobotMsg>::SharedPtr& robot_states_pub)
{
    jaka_msgs::msg::RobotMsg robot_states;
    // RobotStatus robotstatus;
    RobotStatus_simple robotstatus_simple;
    ProgramState programstate;
    BOOL in_pos = true;
    BOOL in_col = false;
    BOOL drag_mode = false;
    BOOL emergency_stop = false;
    robot.is_in_pos(&in_pos);
    robot.is_in_collision(&in_col);
    robot.is_in_drag_mode(&drag_mode);
    robot.is_in_estop(&emergency_stop);
    // robot.get_robot_status(&robotstatus);
    robot.get_robot_status_simple(&robotstatus_simple);
    robot.get_program_state(&programstate);

    // if(robotstatus.emergency_stop)
    if(emergency_stop)
    {
        robot_states.motion_state = 2;
    }
    // else if(robotstatus.errcode)
    else if(robotstatus_simple.errcode)
    {
        robot_states.motion_state = 4;
    }
    // else if(in_pos && programstate == PROGRAM_IDLE && (!robotstatus.drag_status))
    else if(in_pos && programstate == PROGRAM_IDLE && (!drag_mode))
    {
        robot_states.motion_state = 0;
    }
    else if(programstate == PROGRAM_PAUSED)
    {
        robot_states.motion_state = 1;
    }
    // else if((!in_pos) || programstate == PROGRAM_RUNNING || robotstatus.drag_status)
    else if((!in_pos) || programstate == PROGRAM_RUNNING || drag_mode)
    {
        robot_states.motion_state = 3;
    }

    // if(robotstatus.powered_on)
    if(robotstatus_simple.powered_on)
    {
        robot_states.power_state = 1;
 
    }
    else
    {
        robot_states.power_state = 0;
    }

    // if(robotstatus.enabled)
    if(robotstatus_simple.enabled)
    {
        robot_states.servo_state = 1;
    }
    else
    {
        robot_states.servo_state = 0;
    }

    if(in_col)
    {
        robot_states.collision_state = 1;
    }
    else
    {
        robot_states.collision_state = 0;
    }
    robot_states_pub->publish(robot_states);
}

void stop_jog_callback()
{
     if (jog_count >= 1 && jog_count_temp == jog_count )
    {
        robot.jog_stop(-1);
        jog_count = 0;
        jog_count_temp = 0;
        jog_index_last = -1;
        RCLCPP_INFO(rclcpp::get_logger("stop_jog_callback"), "jog stop");
        
    }
    jog_count_temp = jog_count;
}

void get_conn_scoket_state(){
    // RobotStatus robot_status;
	JointValue temp_joints;

    while (rclcpp::ok())
    {
        // int ret = robot.get_robot_status(&robot_status);
        int ret = robot.get_joint_position(&temp_joints);

		// if (ret)
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("get_conn_socket_state"), "get_robot_status error!!!");
        // }
        // else if(!robot_status.is_socket_connect)
		// {
        //     RCLCPP_ERROR(rclcpp::get_logger("get_conn_socket_state"), "connect error!!!");
        // }

        if (ret)
        {
            RCLCPP_ERROR(rclcpp::get_logger("get_conn_socket_state"), 
                         "Connection error or get_joint_position failed, error_code: %d, error: %s", ret, mapErr[ret].c_str());
        }

        if(ret==0)
        {
            // RCLCPP_INFO(rclcpp::get_logger("get_conn_scoket_state"), "ret=0");


            tool_position_callback(tool_position_pub);
            joint_position_callback(joint_position_pub);
            robot_states_callback(robot_state_pub);
        
        }
        rclcpp::sleep_for(chrono::milliseconds(100)); 
    }    
}

void re_enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;

    auto now = std::chrono::steady_clock::now();
    if (is_reset_running) {
        RCLCPP_WARN(rclcpp::get_logger("jaka_driver"), "⚠️ 上一次重置还未完成，忽略本次触发");
        return;
    }
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_reset_time).count() < 3000) {
        RCLCPP_WARN(rclcpp::get_logger("jaka_driver"), "⚠️ 距离上次重置太近，忽略本次触发");
        return;
    }

    is_reset_running = true;
    last_reset_time = now;

    RCLCPP_INFO(rclcpp::get_logger("jaka_driver"), "📥 接收到重新使能请求");

    // 开启一个线程异步执行恢复动作，避免阻塞回调线程
    std::thread([](){
        if (robot.enable_robot() != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("jaka_driver"), "❌ 恢复流程失败（使能失败）");
            is_reset_running = false;
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("jaka_driver"), "✅ 已成功重新使能");

        auto request = std::make_shared<jaka_msgs::srv::Move::Request>();
        request->pose = {1.5,-0.763,-1.176,-0.159,-1.0,-0.67};
        request->has_ref = false;
        request->ref_joint = {0};
        request->mvvelo = 5;
        request->mvacc = 5;
        request->mvtime = 0.0;
        request->mvradii = 0.0;
        request->coord_mode = 0;
        request->index = 0;

        auto future = joint_move_client->async_send_request(request,
            [](rclcpp::Client<jaka_msgs::srv::Move>::SharedFuture response) {
                RCLCPP_INFO(rclcpp::get_logger("jaka_driver"), "🏁 JointMove 已响应（异步）");
                is_reset_running = false;
            });

        // 如果服务失败，3秒后强制释放锁
        std::this_thread::sleep_for(std::chrono::seconds(3));
        is_reset_running = false;
    }).detach();
}




int main(int argc, char *argv[])
{

    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("jaka_driver");
    rclcpp::Rate rate(125); 
    // robot.login_in(argv[1]);
    string default_ip = "10.5.5.100";
    string robot_ip = node->declare_parameter("ip", default_ip);
    robot.login_in(robot_ip.c_str(), false);
    robot.set_status_data_update_time_interval(100);
    robot.set_block_wait_timeout(120);
    robot.power_on();
    sleep(8);
    robot.enable_robot();
    sleep(4);
    //Joint-space first-order low-pass filtering in robot servo mode
    //robot.servo_move_use_joint_LPF(2);
    robot.servo_speed_foresight(15,0.03);

    
    auto gripper_init_service = node->create_service<jaka_msgs::srv::ServoMoveEnable>("/jaka_driver/gripper_init", &gripper_init_enable_callback);

    auto gripper_control_service = node->create_service<jaka_msgs::srv::ServoMoveEnable>("/jaka_driver/gripper_control", &gripper_set_callback);
    //1.1 Linear motion (in customized user coordinate system)
    auto linear_move_service = node->create_service<jaka_msgs::srv::Move>("/jaka_driver/linear_move", &linear_move_callback);
    //1.2 Joint motion
    auto joint_move_service = node->create_service<jaka_msgs::srv::Move>("/jaka_driver/joint_move", &joint_move_callback);
    //1.3 Jog motion
    auto jog_service = node->create_service<jaka_msgs::srv::Move>("/jaka_driver/jog", &jog_callback);
    //1.4 Servo Position Control Mode Enable
    auto servo_move_enable_service = node->create_service<jaka_msgs::srv::ServoMoveEnable>("/jaka_driver/servo_move_enable", &servo_move_enable_callback);
    //1.5 Servo-mode motion in Cartesian space
    auto servo_p_service = node->create_service<jaka_msgs::srv::ServoMove>("/jaka_driver/servo_p", &servo_p_callback);
    //1.6 Joint space servo mode motion
    auto servo_j_service = node->create_service<jaka_msgs::srv::ServoMove>("/jaka_driver/servo_j", &servo_j_callback);
    //1.7 stop motion
    auto stop_move_service = node->create_service<std_srvs::srv::Empty>("/jaka_driver/stop_move", &stop_move_callback);
    //2.1 Setting tcp parameters
    auto set_toolframe_service = node->create_service<jaka_msgs::srv::SetTcpFrame>("/jaka_driver/set_toolframe", &set_toolFrame_callback);
    //2.2 Setting user coordinate system parameters
    auto set_userframe_service = node->create_service<jaka_msgs::srv::SetUserFrame>("/jaka_driver/set_userframe", &set_userFrame_callback);
    //2.3 Set the center of gravity parameters of the robot arm load
    auto set_payload_service = node->create_service<jaka_msgs::srv::SetPayload>("/jaka_driver/set_payload", &set_payload_callback);
    //2.4 Set free drive mode
    auto drag_move_service = node->create_service<std_srvs::srv::SetBool>("/jaka_driver/drag_move", &drag_mode_callback);
    //2.5 Set collision sensitivity
    auto set_crobotollisionlevel_service = node->create_service<jaka_msgs::srv::SetCollision>("/jaka_driver/set_collisionlevel", &set_collisionLevel_callback);
    //2.6 Set IO
    auto set_io_service = node->create_service<jaka_msgs::srv::SetIO>("jaka_driver/set_io",&set_io_callback);
    //2.7 Get IO
    auto get_io_service = node->create_service<jaka_msgs::srv::GetIO>("jaka_driver/get_io",&get_io_callback);
    //2.8 Find the positive solution
    auto get_fk_service = node->create_service<jaka_msgs::srv::GetFK>("jaka_driver/get_fk", &get_fk_callback);
    //2.9 Find the inverse solution
    auto get_ik_service = node->create_service<jaka_msgs::srv::GetIK>("jaka_driver/get_ik", &get_ik_callback);

    // //3.1 End position pose status information reporting
    tool_position_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/jaka_driver/tool_position", 10);
    // //3.2 Joint status information reporting
    joint_position_pub = node->create_publisher<sensor_msgs::msg::JointState>("/jaka_driver/joint_position", 10);
    // //3.3 Report robot event status information
    robot_state_pub = node->create_publisher<jaka_msgs::msg::RobotMsg>("/jaka_driver/robot_states", 10);
    joint_move_client = node->create_client<jaka_msgs::srv::Move>("/jaka_driver/joint_move");
    joint_move_client->wait_for_service();

    // 监听 /teleop/enable 触发重新登录回调
    re_enable_sub = node->create_subscription<std_msgs::msg::Bool>(
        "/teleop/enable", 10, re_enable_callback);
    // Automatically stop robot jog and motion

    
    
    auto stop_jog = node->create_wall_timer(chrono::seconds(3), stop_jog_callback);

    // Monitor network connection status
    thread conn_state_thread(get_conn_scoket_state);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start");

    rclcpp::spin(node);
     // Ensure thread is joined before shutting down the node
    if (conn_state_thread.joinable()) {
        conn_state_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}