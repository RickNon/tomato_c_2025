#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include<sensor_msgs/Joy.h>

using namespace dynamixel;

// Control table address for protocol 2 (MX, XC)
#define ADDR_OPERATING_MODE_P2   11
#define ADDR_TORQUE_ENABLE_P2    64
#define ADDR_GOAL_CURRENT_P2     102
#define ADDR_GOAL_VELOCITY_P2    104
#define ADDR_GOAL_POSITION_P2    116
#define ADDR_PRESENT_CURRENT_P2 126
#define ADDR_PRESENT_VELOCITY_P2 128
#define ADDR_PRESENT_POSITION_P2 132

// Control table address for protocol 1 (AX)
#define ADDR_TORQUE_ENABLE_P1    24
#define ADDR_GOAL_POSITION_P1    30
#define ADDR_PRESENT_POSITION_P1 36

// Protocol version
#define PROTOCOL_VERSION1      1.0
#define PROTOCOL_VERSION2      2.0

// Default setting
#define DXL_AX1_ID            1
#define DXL_AX2_ID            2
#define DXL_AX3_ID            3
#define DXL_AX4_ID            4
#define DXL_AX5_ID            5
#define DXL_MX_ID             10
#define DXL_XC_ID             20
#define BAUDRATE              1000000

#define CURRENT_MODE          0
#define VELOCITY_MODE         1
#define POSITION_MODE         3

#define NODE_FREQUENCY        200

PortHandler * portHandler;
PacketHandler * packetHandler1;
PacketHandler * packetHandler2;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;
uint16_t position_ax_read = 0;
uint16_t position_ax_write = 512; // 0~1023

int16_t vel_mx_read = 0;
int16_t vel_mx_write = 0; // -285 ~ 285

float scale_ax = 4.0;
float vel_ax = 0.0;

float scale_mx = 300.0;
float vel_mx = 0.0;

void joyCallback(const sensor_msgs::Joy& msg)
{
  vel_ax = msg.axes[0]*scale_ax;
  vel_mx_write = msg.axes[1]*scale_mx;      

  if (vel_mx_write > 250){
    vel_mx_write = 250;
  }else if (vel_mx_write  < -250) {
    vel_mx_write = -250;
  }
}
double inverse_sin_0_to_pi(double x) {
    if (x >= 0)
        return std::asin(x);           // x ∈ [0,1], θ ∈ [0, π/2]
    else
        return M_PI + std::asin(x);    // x ∈ [-1,0), θ ∈ (π/2, π]
}
double atan_0_to_pi(double y, double x) {
    double theta = std::atan2(y, x);
    if (theta < 0) theta += M_PI * 2;
    if (theta > M_PI) theta = 2 * M_PI - theta;
    return theta; // θ ∈ [0, π]
}
struct Result{
  double A_angle_0;
  double A_angle_0;
  double A_angle_0;
}
Result inversed_kinematics_mukai(double hand_pos_x,double hand_pos_y,double hand_angle){
  l_0=83;
  l_1=83;
  l_2=50;
  double x_2 = hand_pos_x - l_2 * cos(hand_angle);
  double y_2 = hand_pos_y - l_2 * sin(hand_angle);
  double L_02 = sqrt(x_2*x_2 + y_2*y_2);
  // double beta = asin((l_1*l_1+l_0*l_0-L02*L02)/(2*l_1*l_0));
  double sinbeta=(l_1*l_1+l_0*l_0-L02*L02)/(2*l_1*l_0);
  double beta = inverse_sin_0_to_pi(sinbeta);
  double alpha = atan_0_to_pi(y_2/x_2)-asin(l_1*sinbeta/L02);
  double gamma = hand_angle - alpha - beta;
  return {M_PI/2-alpha, beta, gamma};
}


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ax_mx_joy");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate cycle_rate(NODE_FREQUENCY);
  ros::Subscriber subscriber = nh.subscribe("joy", 1, joyCallback);

  std::string dev_name;
  pnh.param<std::string>("dev", dev_name, "/dev/ttyUSB0");

  portHandler = PortHandler::getPortHandler(dev_name.c_str());
  packetHandler1 = PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
  packetHandler2 = PacketHandler::getPacketHandler(PROTOCOL_VERSION2);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler, DXL_AX1_ID, ADDR_TORQUE_ENABLE_P1, 1, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    //ROS_INFO("Success to enable torque for Dynamixel ID %d", DXL_AX1_ID); 
  }else{
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL_AX1_ID);
    return -1;
  }

  //dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_OPERATING_MODE_P2, CURRENT_MODE, &dxl_error);
  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_OPERATING_MODE_P2, VELOCITY_MODE, &dxl_error);
  //dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_OPERATING_MODE_P2, POSITION_MODE, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    // ROS_INFO("Success to change mode for Dynamixel ID %d", DXL_MX_ID); 
  }else{
    ROS_ERROR("Failed to change mode for Dynamixel ID %d", DXL_MX_ID);
    return -1;
  }

  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_TORQUE_ENABLE_P2, 1, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    // ROS_INFO("Success to enable torque for Dynamixel ID %d", DXL_MX_ID); 
  }else{
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL_MX_ID);
    return -1;
  }

  while(ros::ok())
  {
    ros::spinOnce();

    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    
    ///* AX position mode
    dxl_comm_result = packetHandler1->write2ByteTxRx(portHandler, DXL_AX1_ID, ADDR_GOAL_POSITION_P1, position_ax_write, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      //ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL_AX1_ID, position_write);
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
     
    dxl_comm_result = packetHandler1->read2ByteTxRx(portHandler, DXL_AX1_ID, ADDR_PRESENT_POSITION_P1, (uint16_t *)&position_ax_read, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS)
    {
      ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", DXL_AX1_ID, position_ax_read);
    } else {
      ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
    }

    ///* MX velocity mode
    dxl_comm_result = packetHandler2->write4ByteTxRx(portHandler, DXL_MX_ID, ADDR_GOAL_VELOCITY_P2, vel_mx_write, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      //ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL_MX_ID, vel_mx_write);
    } else {
      ROS_INFO("Failed to set position! Result: %d", dxl_comm_result);
    }
   
    dxl_comm_result = packetHandler2->read2ByteTxRx(portHandler, DXL_MX_ID, ADDR_PRESENT_VELOCITY_P2, (uint16_t *)&vel_mx_read, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS)
    {
      ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", DXL_MX_ID, vel_mx_read);
    } else {
      ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    }
    
    position_ax_write = position_ax_write + int(vel_ax);
    if (position_ax_write > 700) position_ax_write = 700;
    else if (position_ax_write  < 300) position_ax_write = 300;

    cycle_rate.sleep();
  }
  
  dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler, DXL_AX1_ID, ADDR_TORQUE_ENABLE_P1, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", DXL_AX1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_TORQUE_ENABLE_P2, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", DXL_MX_ID);
    return -1;
  }

  portHandler->closePort();

  return 0;
}
