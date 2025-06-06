#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>

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
#define DXL_AX1_ID            1 //手先の角度変更
#define DXL_AX2_ID            2 //アーム根本から１
#define DXL_AX3_ID            3 //アーム根本から２
#define DXL_AX4_ID            4 //アーム根本から３
#define DXL_AX5_ID            5 //手先の作動モータ
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

float vel_ax = 0.0;

float scale_mx = 300.0;
float vel_mx = 0.0;

// ピッチ回転の定数
bool  pitch_up     = false;
bool  pitch_down   = false;
float scale_ax     = 4.0;
int pitch_flat = 512;

// アームの移動
float cmd_x = 0;
float cmd_y = 0;

std::vector<int> DXL_AX_ID = {DXL_AX1_ID, DXL_AX2_ID, DXL_AX3_ID, DXL_AX4_ID, DXL_AX5_ID};

std::vector<int> position_ax = {512, 512, 512, 512, 512};

void joyCallback(const sensor_msgs::Joy& msg)
{

  //アーム制御
  ROS_INFO("aaaaaaaaa");
  float _gain_x = -2;
  float _gain_y = 1;
  cmd_x = msg.axes[3] * _gain_x;
  cmd_y = msg.axes[4] * _gain_y;
  // Y:3 A: 0
  // 手先のピッチ回転
  // vel_ax = msg.axes[0]*scale_ax;
  if(msg.buttons[0] == 1 && msg.buttons[3] == 1){
    pitch_up   = false;
    pitch_down = false;
  } else if(msg.buttons[3] == 1){
    pitch_up = true;
  } else if(msg.buttons[0] == 1){
    pitch_down = true;
  }

  if(msg.buttons[3] == 0){
    pitch_up = false;
  }

  // 昇降機
  vel_mx_write = msg.axes[7]*scale_mx;      

  if (vel_mx_write > 250){
    vel_mx_write = 250;
  }else if (vel_mx_write  < -250) {
    vel_mx_write = -250;
  }
}

//手先のピッチ回転
void hand_picth(){
  // Yを押したら手先が上を向く
  if(pitch_up){
    position_ax_write = position_ax_write + scale_ax*3.0;
  }

  if (position_ax_write < pitch_flat) position_ax_write = pitch_flat;
  // Aを押したら指定位置に戻る
  while(pitch_down){
  if(position_ax_write < pitch_flat){
    pitch_down = false;
  } 
  position_ax_write = position_ax_write - scale_ax;
  position_ax[0] = position_ax_write;
 }

}


float inverse_sin_0_to_pi(float x) {
  if (x >= 0)
      return std::asin(x);           // x ∈ [0,1], θ ∈ [0, π/2]
  else
      return M_PI + std::asin(x);    // x ∈ [-1,0), θ ∈ (π/2, π]
}
float atan_0_to_pi(float y, float x) {
  if(x == 0) return M_PI/2;
  float theta = std::atan2(y, x);
  if (theta < 0) theta += M_PI * 2;
  if (theta > M_PI) theta = 2 * M_PI - theta;
  return theta; // θ ∈ [0, π]
}
struct Result{
  float A_angle_2;
  float A_angle_3;
  float A_angle_4;
};

Result inversed_kinematics_mukai(float hand_pos_x, float hand_pos_y, float hand_angle){
  ROS_INFO("%lf %lf %lf", hand_pos_x, hand_pos_y, hand_angle);
  float l_0=83;
  float l_1=83;
  float l_2=0;
  float x_2 = hand_pos_x - l_2 * cos(hand_angle);
  float y_2 = hand_pos_y - l_2 * sin(hand_angle);
  float L_02 = sqrt(x_2*x_2 + y_2*y_2);
  if(L_02 > l_0 + l_1) L_02 = l_0+l_1-1;
  // float beta = asin((l_1*l_1+l_0*l_0-L02*L02)/(2*l_1*l_0));
  
  float cosbeta=-(l_1*l_1+l_0*l_0-L_02*L_02)/(2*l_1*l_0);
  // float beta = inverse_sin_0_to_pi(sinbeta);
  float beta = acos(cosbeta);
  float sinbeta=sin(beta);
  float alpha = atan_0_to_pi(y_2, x_2)-asin(l_1*sinbeta/L_02);
  float gamma = hand_angle - alpha - beta;
  ROS_INFO("position123 %lf, position2 %lf, position3 %lf", alpha, beta, gamma);
  return {alpha - float(M_PI)/2, beta, gamma};
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ax_mx_custom");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate cycle_rate(NODE_FREQUENCY);
  ros::Subscriber subscriber = nh.subscribe("joy", 1, joyCallback);

  //手先先端の初期位置を定義
  geometry_msgs::PointStamped target_point;
  target_point.point.x = 83;
  target_point.point.y = 83;

  std::string dev_name;
  pnh.param<std::string>("dev", dev_name, "/dev/ttyUSB0");

  portHandler = PortHandler::getPortHandler(dev_name.c_str());
  packetHandler1 = PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
  packetHandler2 = PacketHandler::getPacketHandler(PROTOCOL_VERSION2);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!!!!!!!!!!!!!!");
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
  
    // //AXモータの値を書き込むやつ
    for(int i = 0; i < DXL_AX_ID.size(); i++){
      dxl_comm_result = packetHandler1->write2ByteTxRx(portHandler, DXL_AX_ID[i], ADDR_GOAL_POSITION_P1, position_ax[i], &dxl_error);
      if (dxl_comm_result == COMM_SUCCESS) {
        //ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL_AX_ID, position_write);
      } else {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
      }
     
      dxl_comm_result = packetHandler1->read2ByteTxRx(portHandler, DXL_AX_ID[i], ADDR_PRESENT_POSITION_P1, (uint16_t *)&position_ax_read, &dxl_error);
      if (dxl_comm_result == COMM_SUCCESS)
      {
        // ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", DXL_AX_ID, position_ax_read);
      } else {
        ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
      }
    }



    ///* MX velocity mode
    dxl_comm_result = packetHandler2->write4ByteTxRx(portHandler, DXL_MX_ID, ADDR_GOAL_VELOCITY_P2, vel_mx_write, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      //ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL_MX_ID, vel_mx_write);
    } else {
      // ROS_INFO("Failed to set position! Result: %d", dxl_comm_result);
    }
   
    dxl_comm_result = packetHandler2->read2ByteTxRx(portHandler, DXL_MX_ID, ADDR_PRESENT_VELOCITY_P2, (uint16_t *)&vel_mx_read, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS)
    {
      // ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", DXL_MX_ID, vel_mx_read);
    } else {
      ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    }

    //手先のピッチ回転
    hand_picth();


    target_point.point.x += cmd_x;
    target_point.point.y += cmd_y;

    Result inversed_result = inversed_kinematics_mukai(target_point.point.x, target_point.point.y, M_PI/2);
    position_ax[1] = int(inversed_result.A_angle_2 / float(M_PI) / 2 * 1024 + 512);
    position_ax[2] = int(inversed_result.A_angle_3 / float(M_PI) / 2 * 1024 + 512);
    position_ax[3] = int(inversed_result.A_angle_4 / float(M_PI) / 2 * 1024 + 512);
    // ROS_INFO("position1 %lf, position2 %lf, position3 %lf", inversed_result.A_angle_2, inversed_result.A_angle_3, inversed_result.A_angle_4);
    ROS_INFO("position1 %d, position2 %d, position3 %d", position_ax[1], position_ax[2], position_ax[3]);
    // ROS_INFO("")
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
