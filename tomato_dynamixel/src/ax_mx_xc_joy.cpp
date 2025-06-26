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
#define C_BASED_P_MODE        5

#define NODE_FREQUENCY        200
#define TORQUE_STOP_THRESHOLD  80 

bool torque_exceeded = false;


PortHandler * portHandler;
PacketHandler * packetHandler1;
PacketHandler * packetHandler2;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;
uint16_t position_ax_read = 0;
uint16_t position_ax_write = 512; // 0~1023


int16_t vel_mx_read = 0;
int16_t vel_mx_write = 0; // -285 ~ 285

uint16_t position_xc_read = 0;
int32_t goal_position_xc = 0; // 中央（0~4095+）１回転（）
uint16_t torque_limit_ma = 50;  // 0〜920
uint16_t raw_current=0;
int16_t present_current=0;



float scale_ax = 4.0;
float vel_ax = 0.0;

float scale_mx = 300.0;
float vel_mx = 0.0;

float scale_xc = 20.0;
float vel_xc = 0.0;




void joyCallback(const sensor_msgs::Joy& msg)
{
  vel_ax = msg.axes[0]*scale_ax;
  vel_mx_write = msg.axes[1]*scale_mx;   
  // vel_xc = (msg.axes[5]-msg.axes[2])*scale_xc;   
  if(msg.axes[5] < 0.8 && msg.axes[2] < 0.8){
    vel_xc = 0.0;
  }else if(msg.axes[5] < 0.8){
    vel_xc = scale_xc;
  }else if(msg.axes[2] < 0.8){
    vel_xc = -scale_xc;
  }else{
    vel_xc = 0.0;
  }

   
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ax_mx_xc_joy");
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
  //AX1 トルクON
  dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler, DXL_AX1_ID, ADDR_TORQUE_ENABLE_P1, 1, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    //ROS_INFO("Success to enable torque for Dynamixel ID %d", DXL_AX1_ID); 
  }else{
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL_AX1_ID);
    return -1;
  }


  //MXモードセット
  //dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_OPERATING_MODE_P2, CURRENT_MODE, &dxl_error);
  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_OPERATING_MODE_P2, VELOCITY_MODE, &dxl_error);
  //dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_OPERATING_MODE_P2, POSITION_MODE, &dxl_error);

  if (dxl_comm_result == COMM_SUCCESS) {
    // ROS_INFO("Success to change mode for Dynamixel ID %d", DXL_MX_ID); 
  }else{
    ROS_ERROR("Failed to change mode for Dynamixel ID %d", DXL_MX_ID);
    return -1;
  }


  //XCモードセット
  //dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_XC_ID, ADDR_OPERATING_MODE_P2, CURRENT_MODE, &dxl_error);
  //dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_XC_ID, ADDR_OPERATING_MODE_P2, VELOCITY_MODE, &dxl_error);
  //dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_XC_ID, ADDR_OPERATING_MODE_P2, POSITION_MODE, &dxl_error);
  dxl_comm_result =packetHandler2->write1ByteTxRx(portHandler, DXL_XC_ID, ADDR_OPERATING_MODE_P2, C_BASED_P_MODE, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    // ROS_INFO("Success to change mode for Dynamixel ID %d", DXL_XC_ID); 
  }else{
    ROS_ERROR("Failed to change mode for Dynamixel ID %d", DXL_XC_ID);
    return -1;
  }

  //MXトルクON
  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_TORQUE_ENABLE_P2, 1, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    // ROS_INFO("Success to enable torque for Dynamixel ID %d", DXL_MX_ID); 
  }else{
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL_MX_ID);
    return -1;
  }

  //XCトルク制御
  dxl_comm_result = packetHandler2->write2ByteTxRx(
    portHandler, DXL_XC_ID, ADDR_GOAL_CURRENT_P2, torque_limit_ma, &dxl_error);
if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to set torque limit");
    return -1;
}

  //XCトルクON
  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_XC_ID, ADDR_TORQUE_ENABLE_P2, 1, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    // ROS_INFO("Success to enable torque for Dynamixel ID %d", DXL_XC_ID); 
  }else{
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL_XC_ID);
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
     // ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", DXL_AX1_ID, position_ax_read);
    } else {
      ROS_ERROR("AXFailed to get position! Result: %d", dxl_comm_result);
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
      // ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", DXL_MX_ID, vel_mx_read);
    } else {
      ROS_INFO("MXFailed to get position! Result: %d", dxl_comm_result);
    }

    ///* XC C based P mode
    
    
    //位置書き込み
    dxl_comm_result = packetHandler2->write4ByteTxRx(portHandler, DXL_XC_ID, ADDR_GOAL_POSITION_P2, goal_position_xc, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL_XC_ID, goal_position_xc);
    } else {
      ROS_INFO("Failed to set position! Result: %d", dxl_comm_result);
    }

    //位置読み込み
    dxl_comm_result = packetHandler2->read2ByteTxRx(portHandler, DXL_XC_ID, ADDR_PRESENT_POSITION_P2, (uint16_t *)&position_xc_read, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS)
    {
    //  ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", DXL_XC_ID, position_xc_read);
    } else {
      ROS_ERROR("XCFailed to get position! Result: %d", dxl_comm_result);
    }


    //電流読み取り
    //dxl_comm_result = packetHandler2->read2ByteTxRx(portHandler, DXL_XC_ID, ADDR_PRESENT_CURRENT_P2, (uint16_t *)&cur_xc_read, &dxl_error);

    // dxl_comm_result = packetHandler2->read2ByteTxRx(portHandler, DXL_XC_ID, ADDR_PRESENT_CURRENT_P2, &raw_current, &dxl_error);
    // present_current = static_cast<int16_t>(raw_current);


    // if (dxl_comm_result == COMM_SUCCESS)
    // {
    //   // ROS_INFO("getCurrent : [ID:%d] -> [CURRENT:%d]", DXL_XC_ID, present_current);
    //     if (abs(present_current) > TORQUE_STOP_THRESHOLD) {
    //       ROS_WARN("Torque exceeded! Stopping motion.");
    //         torque_exceeded = true;
    //     }else{
    //       torque_exceeded = false;
    //     }

    // } else {
    //   ROS_INFO("Failed to get Current! Result: %d", dxl_comm_result);
    // }

    goal_position_xc += static_cast<int32_t>(vel_xc);
    ROS_INFO("goal position xc: %d", goal_position_xc);

    if (goal_position_xc > 700) goal_position_xc = 700;
    if (goal_position_xc < 0)    goal_position_xc = 0;


    
    position_ax_write = position_ax_write + int(vel_ax);
    if (position_ax_write > 700) position_ax_write = 700;
    else if (position_ax_write  < 300) position_ax_write = 300;

    // if (!torque_exceeded) {

    //   goal_position_xc += static_cast<int32_t>(vel_xc);
    //   ROS_INFO("goal position xc: %d", goal_position_xc);

    //   if (goal_position_xc > 700) goal_position_xc = 700;
    //   if (goal_position_xc < 0)    goal_position_xc = 0;

    // } else {
    //   // 動かさない（goal_position_xc更新なし）
    // }

    cycle_rate.sleep();
  }
  

  //AXトルクOFF
  dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler, DXL_AX1_ID, ADDR_TORQUE_ENABLE_P1, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", DXL_AX1_ID);
    return -1;
  }
  //MX トルクOFF
  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_TORQUE_ENABLE_P2, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", DXL_MX_ID);
    return -1;
  }

  //XC トルクOFF
  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_XC_ID, ADDR_TORQUE_ENABLE_P2, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", DXL_XC_ID);
    return -1;
  }

  portHandler->closePort();

  return 0;
}
