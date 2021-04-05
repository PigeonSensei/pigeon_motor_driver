#ifndef L298N_H
#define L298N_H

#include <ros/ros.h>

#include "l298n/MotorCommand.h"
#include "l298n/SetMotorDriver.h"
#include <geometry_msgs/Twist.h>

#include <wiringPi.h>
#include <softPwm.h>

class L298N
{
public:
    L298N(ros::NodeHandle &n)
      : subscriber_cmd_vel_(n.subscribe("cmd_vel", 100, &L298N::CmdVelCallback, this)),
        subscriber_motor_command_(n.subscribe("motor_command", 100, &L298N::MotorCommandCallback, this)),
        service_server_set_motor_driver_(n.advertiseService("SetMotorDriver", &L298N::SetMotorDriverServiceCallback, this))
       {
          // open run
          ROS_INFO("l298n_node Open");
          GetROSParam(n);
       }
       ~L298N()
       {
          // close run
          ROS_INFO("l298n_node Close");
          DoStop();
       }

    struct Switch
    {
      int emergency_stop = 0;
    };

    int SetupPin();

    int GetROSParam(ros::NodeHandle &n);

    bool SetMotorDriverServiceCallback(l298n::SetMotorDriver::Request &req,
                                      l298n::SetMotorDriver::Response &res);

    void MotorCommandCallback(const l298n::MotorCommand &data);

    int DoStop();

    int DoGo(int rpm_L, int rpm_R);

    int Spin();

    void CmdVelCallback(const geometry_msgs::Twist &cmd_vel);



private:

  int pin_IN1_;
  int pin_IN2_;
  int pin_IN3_;
  int pin_IN4_;
  int pin_ENA_;
  int pin_ENB_;

  int min_value_;

  Switch switch_;

  l298n::MotorCommand motor_command_;

  ros::Subscriber subscriber_cmd_vel_;
  ros::Subscriber subscriber_motor_command_;

  ros::ServiceServer service_server_set_motor_driver_;

};

#endif // L298N_H
