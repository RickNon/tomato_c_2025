#ifndef AX_MX_CUSTOM_HPP
#define AX_MX_CUSTOM_HPP

// Include ROS and message libraries
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include "std_msgs/String.h"
#include <vector>
#include <cmath>

// Include Dynamixel SDK
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <dynamixel_sdk/group_sync_write.h>

// Use the dynamixel namespace for SDK classes
namespace dynamixel {
  // Forward declarations of SDK handler classes
  class PortHandler;
  class PacketHandler;
}

#define PROTOCOL_VERSION1 1.0    // AX protocol
#define PROTOCOL_VERSION2 2.0    // MX/XC protocol

// Default Dynamixel IDs
#define DXL_AX1_ID       1      // Wrist servo
#define DXL_AX2_ID       2      // Base joint 1
#define DXL_AX3_ID       3      // Base joint 2
#define DXL_AX4_ID       4      // Base joint 3

#define DXL_MX_ID        10     // Gripper motor

#define BAUDRATE         1000000

// Operating modes
#define CURRENT_MODE     0
#define VELOCITY_MODE    1
#define POSITION_MODE    3

// Node update frequency (Hz)
#define NODE_FREQUENCY   200

// Control table addresses for MX (protocol 2)
#define ADDR_OPERATING_MODE_P2   11
#define ADDR_TORQUE_ENABLE_P2    64
#define ADDR_GOAL_CURRENT_P2     102
#define ADDR_GOAL_VELOCITY_P2    104
#define ADDR_GOAL_POSITION_P2    116
#define ADDR_PRESENT_CURRENT_P2  126
#define ADDR_PRESENT_VELOCITY_P2 128
#define ADDR_PRESENT_POSITION_P2 132

// Control table addresses for AX (protocol 1)
#define ADDR_TORQUE_ENABLE_P1    24
#define ADDR_GOAL_POSITION_P1    30
#define ADDR_PRESENT_POSITION_P1 36
#define ADDR_MOVING_SPEED_P1     32

extern dynamixel::PortHandler *portHandler;       // Serial port handler
extern dynamixel::PacketHandler *packetHandler1;  // AX packet handler
extern dynamixel::PacketHandler *packetHandler2;  // MX packet handler

extern uint8_t dxl_error;           // Error byte from SDK
extern int dxl_comm_result;         // Communication result

extern uint16_t position_ax_read;   // Read-back position from AX servos
extern uint16_t position_ax_write;  // Commanded position for AX servos

extern int16_t vel_mx_write;        // Commanded velocity for MX servo
extern int16_t vel_mx_read;         // Read-back velocity from MX servo

extern float vel_ax;                // Velocity command for AX servos (unused)
extern float scale_mx;              // Scaling factor for MX velocity
extern float vel_mx;                // Measured MX velocity

// Flags and scaling for wrist pitch control
extern bool pitch_up;
extern bool pitch_down;
extern float scale_ax;
extern int pitch_flat;

// Arm movement commands
extern float cmd_x;
extern float cmd_y;

extern std::vector<int> DXL_AX_ID;  // List of AX servo IDs
extern std::vector<int> position_ax; // Target positions for AX servos

/**
 * @brief Compute atan2 in range [0, π]
 * @param y y-coordinate
 * @param x x-coordinate
 * @return angle in radians between 0 and π
 */
float atan_0_to_pi(float y, float x);

/**
 * @brief Compute inverse sine in range [0, π]
 * @param x input in [-1, 1]
 * @return angle in radians between 0 and π
 */
float inverse_sin_0_to_pi(float x);

/**
 * @struct InverseAngles
 * @brief Holds output angles for inverse kinematics
 */
struct InverseAngles {
  float A_angle_2;
  float A_angle_3;
  float A_angle_4;
};

/**
 * @brief Compute inverse kinematics (x-y plane)
 * @param hand_pos_x x-coordinate of the end-effector
 * @param hand_pos_y y-coordinate of the end-effector
 * @param hand_angle orientation angle of the end-effector
 * @return Result containing joint angles
 */
InverseAngles inversed_kinematics(float hand_pos_x, float hand_pos_y, float hand_angle);

/**
 * @brief Adjust wrist pitch based on pitch flags
 */
void hand_picth();  // Note: spelling preserved from .cpp

/**
 * @brief Callback for joystick input to control arm and gripper
 * @param msg Joy message with axes and buttons
 */
void joyCallback(const sensor_msgs::Joy &msg);

#endif // AX_MX_CUSTOM_HPP
