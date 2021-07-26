#ifndef L298N_H
#define L298N_H

#include <ros/ros.h>

#include "motor_driver_msgs/MotorCommand.h"
#include "l298n/SetMotorDriver.h"

#include <wiringPi.h>
#include <softPwm.h>

class L298N
{
public:
    L298N(ros::NodeHandle &n)
      : subscriber_motor_command_(n.subscribe("motor_command", 100, &L298N::MotorCommandCallback, this)),
        service_server_set_motor_driver_(n.advertiseService("SetMotorDriver", &L298N::SetMotorDriverServiceCallback, this))
       {
          // open run
          ROS_INFO("l298n_node Open");
          GetROSParam();
          SetupPin();
       }
       ~L298N()
       {
          // close run
          ROS_INFO("l298n_node Close");
          DoStop();
       }

    int SetupPin();

    int GetROSParam();

    bool SetMotorDriverServiceCallback(l298n::SetMotorDriver::Request &req,
                                      l298n::SetMotorDriver::Response &res);

    void MotorCommandCallback(const motor_driver_msgs::MotorCommand &data);

    int DoStop();

    int DoGo(int command_L, int command_R);

    int Spin();


private:

  int pin_IN1_;
  int pin_IN2_;
  int pin_IN3_;
  int pin_IN4_;
  int pin_ENA_;
  int pin_ENB_;

  int minimum_motor_command_;
  int maximum_motor_command_;

  struct Switch
  {
    int emergency_stop = 0;
  };

  Switch switch_;

  motor_driver_msgs::MotorCommand motor_command_;

  ros::Subscriber subscriber_motor_command_;

  ros::ServiceServer service_server_set_motor_driver_;

};

#endif // L298N_H
