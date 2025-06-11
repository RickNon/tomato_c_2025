#include "../include/kobuki_operation/kobuki_operation.hpp"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

double _g_speed = 0.2;
double _g_turn = 1;
double motion_v_tmp   = 0.0;
double rotation_v_tmp = 0.0;

geometry_msgs::Twist command;
    

KobukiOperation::KobukiOperation(double freq) :
    _nh(),
    _freq(freq),
    _update_rate(freq),
    _control{0},
    _speed_acc(0.005),
    _turn_acc(0.6)

{
    _joy_sub = _nh.subscribe("joy",10,&KobukiOperation::joy_callback,this);
    _cmd_sub = _nh.subscribe("cmd_vel", 10, &KobukiOperation::cmd_callback, this);
    _kobuki_pub = _nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);   
}

void KobukiOperation::joy_callback(const sensor_msgs::Joy &joy_msg)
{
    double _g_turn = 1;
    double rotation_v_tmp = joy_msg.axes[0] * _g_turn;

    motion_v_tmp = joy_msg.axes[1] * _g_speed;
    if (joy_msg.axes[1] < 0){
        rotation_v_tmp = - joy_msg.axes[0] * _g_turn;
    } else {
        rotation_v_tmp = joy_msg.axes[0] * _g_turn;
    }

    if( (abs(joy_msg.axes[1])<=0.1)&(abs(joy_msg.axes[0])<=0.1) ){
        // std::cout << "Stop" << std::endl;
        kobukiStop();
    }else{
        // std::cout << "Move" << std::endl;
        kobukiMove(motion_v_tmp, rotation_v_tmp);
    }
}

void KobukiOperation::cmd_callback(const geometry_msgs::Twist &cmd_msg)
{
    // Forward raw values to the same ramp filter used for Joy
    kobukiMove(cmd_msg.linear.x, cmd_msg.angular.z);
}

void KobukiOperation::spin()
{
    ///< awaken timer
    ros::Duration(0.1).sleep();

    ///< change terminal setting
    struct termios old_terminal, new_terminal;
    int old_fcntl;
    tcgetattr(STDIN_FILENO, &old_terminal);
    new_terminal = old_terminal;
    new_terminal.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);
    old_fcntl = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_fcntl | O_NONBLOCK);

    while (ros::ok())
    {
        ros::spinOnce();
        normalOperation();
    }
    kobukiStop();

    ///< restore terminal setting
    tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);
    fcntl(STDIN_FILENO, F_SETFL, old_fcntl);
}


void KobukiOperation::kobukiMove(double speed, double turn)
{
    _control.target_speed = speed;
    _control.target_turn = turn;
    kobukiInterpolate();
}


void KobukiOperation::kobukiStop()
{
    _control.target_speed = 0.0;
    _control.target_turn = 0.0;
    kobukiInterpolate();
}

void KobukiOperation::normalOperation()
{
    kobukiInterpolate();
    _update_rate.sleep();
  
}

void KobukiOperation::kobukiInterpolate()
{
     // 線形速度の補間（加速度制限）
 
    double speed_diff = _control.target_speed - _control.control_speed;
    double turn_diff = _control.target_turn - _control.control_turn;

    // 加速度制限の無視
    _no_acc_limit = (std::abs(speed_diff) <= 0.06 && std::abs(turn_diff) <= 0.3);

    if (_no_acc_limit) {
        // 加速度制限なしで即時反映
        _control.control_speed = _control.target_speed;
        _control.control_turn = _control.target_turn;
    } 
    else {
        // 線形速度の補間（加速度制限）
        _control.control_speed += (_speed_acc * (speed_diff > 0 ? 1 : -1));
        _control.control_turn += (_turn_acc * (turn_diff > 0 ? 1 : -1));
    }
    geometry_msgs::Twist command;
    command.linear.x = _control.control_speed;
    command.angular.z = _control.control_turn;
    _kobuki_pub.publish(command);
}

void KobukiOperation::cmdInterpolate()
{
    double speed_diff = _control.target_speed - _control.control_speed;
    double turn_diff  = _control.target_turn  - _control.control_turn;

    _control.control_speed += speed_diff / 5;
    _control.control_turn  += turn_diff  / 10;

    command.linear.x = _control.control_speed;
    command.angular.z = _control.control_turn;
    _kobuki_pub.publish(command);
}
