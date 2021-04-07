#ifndef MOTOR_DRIVER_TESTER_H
#define MOTOR_DRIVER_TESTER_H

#include <ros/ros.h>

#include "motor_driver_msgs/MotorCommand.h"
#include "pigeon_run/MotorCommand.h"

#include <termios.h>

#include "ftxui/component/container.hpp"
#include "ftxui/component/checkbox.hpp"
#include "ftxui/screen/string.hpp"
#include "ftxui/screen/color.hpp"
#include "pigeon_terminal/pigeon_terminal.h"

class Motor_driver_tester
{
public:
    Motor_driver_tester(ros::NodeHandle &n)
      : publisher_motor_command_(n.advertise<pigeon_run::MotorCommand>("motor_command",1000))
       {
          // open run
          ROS_INFO("motor_driver_tester_node OPNE");
       }
       ~Motor_driver_tester()
       {
          // close run
          ROS_INFO("motor_driver_tester_node CLOSE");
       }

    void DrawTUI();

    int SetKey();

    void InputMotorCommand(int cmd_vel_menu_number, int key_input);

    void InputCmdVels(int cmd_vel_menu_number, double cmd_vel);

    void ResetAtMotorCommand(int cmd_vel_menu_number);

    void ResetAllMotorCommand();

    int UpdateMotorCommand_();

    int Publisher();

    void Spin();

    void Exit();

    int key_value_;

private:
    ros::Publisher publisher_motor_command_;
    motor_driver_msgs::MotorCommand motor_command_;
    Pigeon_terminal pigeon_terminal_;

    int menu_number_ = 0;
    int state_mute_ = 0;
    int state_join_ = 0;


};
#endif // MOTOR_DRIVER_TESTER_H
