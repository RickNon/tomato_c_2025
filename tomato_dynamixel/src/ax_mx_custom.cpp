#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include "ax_mx_custom.hpp"  // Include corresponding header

using namespace dynamixel;

PortHandler* portHandler;
PacketHandler* packetHandler1;
PacketHandler* packetHandler2;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;
uint16_t position_ax_read = 0;
uint16_t position_ax_write = 512;   // Default AX goal position (0~1023)

int16_t vel_mx_read = 0;
int16_t vel_mx_write = 0;           // Default MX velocity command

float vel_ax = 0.0f;
float scale_mx = 300.0f;            // Scaling factor for MX motor (based on original)
float vel_mx = 0.0f;

bool pitch_up = false;
bool pitch_down = false;
float scale_ax = 4.0f;              // Scaling factor for AX servos (pitch control)
int pitch_flat = 512;

float cmd_x = 0.0f;
float cmd_y = 0.0f;

const uint16_t MOVING_SPEED_P1 = 200;
GroupSyncWrite* gsync_ax = nullptr;

std::vector<int> DXL_AX_ID = { DXL_AX1_ID, DXL_AX2_ID, DXL_AX3_ID, DXL_AX4_ID };
std::vector<int> position_ax(DXL_AX_ID.size(), position_ax_write);


float atan_0_to_pi(float y, float x) {
    float angle = std::atan2(y, x);
    if (angle < 0) angle += M_PI;
    return angle;
}

float inverse_sin_0_to_pi(float x) {
    if (x >= 0)
        return std::asin(x);           // x ∈ [0,1], θ ∈ [0, π/2]
    else
        return M_PI + std::asin(x);    // x ∈ [-1,0), θ ∈ (π/2, π]
}

InverseAngles inversed_kinematics(float hand_pos_x, float hand_pos_y, float hand_angle) {
  // ROS_INFO("%lf %lf %lf", hand_pos_x, hand_pos_y, hand_angle);
  float l_0 = 83;
  float l_1 = 83;
  float l_2 = 0;
  float x_2 = hand_pos_x - l_2 * cos(hand_angle);
  float y_2 = hand_pos_y - l_2 * sin(hand_angle);
  float L_02 = sqrt(x_2*x_2 + y_2*y_2);
  if(L_02 > l_0 + l_1) L_02 = l_0+l_1-1;
  
  float cosbeta = -(l_1*l_1+l_0*l_0-L_02*L_02)/(2*l_1*l_0);
  float beta = acos(cosbeta);
  float sinbeta = sin(beta);
  float alpha = atan_0_to_pi(y_2, x_2)-asin(l_1*sinbeta/L_02);
  float gamma = hand_angle - alpha - beta;
  // ROS_INFO("position123 %lf, position2 %lf, position3 %lf", alpha, beta, gamma);
  return {alpha - float(M_PI)/2, beta, gamma};
}

void hand_picth() {
    // Rotate wrist pitch up/down based on flags
    if (pitch_up) {
        position_ax_write += scale_ax * 3.0f;
    }
    if (position_ax_write < pitch_flat) position_ax_write = pitch_flat;
    while (pitch_down) {
        if (position_ax_write < pitch_flat) {
            pitch_down = false;
            break;
        }
        position_ax_write -= scale_ax;
        position_ax[0] = position_ax_write;
    }
}

void joyCallback(const sensor_msgs::Joy& msg) {
    // Arm movement control
    const float gain_x = -2.0f;
    const float gain_y = 1.0f;
    cmd_x = msg.axes[3] * gain_x;
    cmd_y = msg.axes[4] * gain_y;

    // Pitch : Y (3) = up while pushing, A (0) = down once
    if (msg.buttons[0] == 1 && msg.buttons[3] == 1) {
        pitch_up   = false;
        pitch_down = false;
    } else if (msg.buttons[3] == 1) {
        pitch_up = true;
    } else if (msg.buttons[0] == 1) {
        pitch_down = true;
    }
    if (msg.buttons[3] == 0) {
        pitch_up = false;
    }

    // Lift control for MX motor (axes[7])
    vel_mx_write = static_cast<int16_t>(msg.axes[7] * scale_mx);
    if (vel_mx_write > 250) {
        vel_mx_write = 250;
    } else if (vel_mx_write < -250) {
        vel_mx_write = -250;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ax_mx_custom_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::Rate rate(NODE_FREQUENCY);

    ros::Subscriber sub = nh.subscribe("joy", 10, joyCallback);
    ros::Publisher pub = nh.advertise<std_msgs::String>("status", 10);

    // Initial end-effector point for IK
    geometry_msgs::PointStamped target_point;
    target_point.point.x = 83;
    target_point.point.y = 83;

    // Get device name parameter
    std::string dev_name;
    pnh.param<std::string>("dev", dev_name, "/dev/ttyUSB0");

    // Initialize port and packet handlers
    portHandler    = PortHandler::getPortHandler(dev_name.c_str());
    packetHandler1 = PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
    packetHandler2 = PacketHandler::getPacketHandler(PROTOCOL_VERSION2);

    // Open and configure serial port
    if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open port");
        return -1;
    }
    if (!portHandler->setBaudRate(BAUDRATE)) {
        ROS_ERROR("Failed to set baudrate");
        return -1;
    }

    // set write handler for AX
    gsync_ax = new GroupSyncWrite(portHandler, packetHandler1,
                                  ADDR_GOAL_POSITION_P1, 2);
    for (uint8_t id : DXL_AX_ID) {
        packetHandler1->write2ByteTxRx(portHandler, id,
                                      ADDR_MOVING_SPEED_P1,
                                      MOVING_SPEED_P1, &dxl_error);
    }

    // Enable torque for AX servo
    dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler, DXL_AX1_ID, ADDR_TORQUE_ENABLE_P1, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to enable torque for AX ID %d", DXL_AX1_ID);
        return -1;
    }

    // Set MX to velocity mode and enable torque
    dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_OPERATING_MODE_P2, VELOCITY_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to set MX operating mode");
        return -1;
    }
    dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_TORQUE_ENABLE_P2, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_ERROR("Failed to enable torque for MX ID %d", DXL_MX_ID);
        return -1;
    }

    // Main control loop
    while (ros::ok()) {
        ros::spinOnce();

        // Write positions to AX servos simulteneously
        gsync_ax->clearParam();
        for (size_t i = 0; i < DXL_AX_ID.size(); ++i) {
            uint8_t param[2] = {
                DXL_LOBYTE(position_ax[i]),
                DXL_HIBYTE(position_ax[i])
            };
            gsync_ax->addParam(DXL_AX_ID[i], param);
        }
        gsync_ax->txPacket();

        // Write velocity to MX motor
        dxl_comm_result = packetHandler2->write4ByteTxRx(portHandler, DXL_MX_ID, ADDR_GOAL_VELOCITY_P2, vel_mx_write, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("Failed to set velocity for MX ID %d", DXL_MX_ID);
        }

        // Update IK target and compute new positions
        target_point.point.x += cmd_x;
        target_point.point.y += cmd_y;
        InverseAngles inv_res = inversed_kinematics(target_point.point.x, target_point.point.y, M_PI/2);
        position_ax[1] = int(inv_res.A_angle_2 / M_PI / 2 * 1024 + 512);
        position_ax[2] = int(inv_res.A_angle_3 / M_PI / 2 * 1024 + 512);
        position_ax[3] = int(inv_res.A_angle_4 / M_PI / 2 * 1024 + 512);
        // ROS_INFO("position1 %d, position2 %d, position3 %d", position_ax[1], position_ax[2], position_ax[3]);

        // Publish status message
        std_msgs::String status_msg;
        status_msg.data = "running";
        pub.publish(status_msg);

        rate.sleep();
    }

    // Disable torque and close port
    packetHandler1->write1ByteTxRx(portHandler, DXL_AX1_ID, ADDR_TORQUE_ENABLE_P1, 0, &dxl_error);
    packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_TORQUE_ENABLE_P2, 0, &dxl_error);
    portHandler->closePort();
    return 0;
}
