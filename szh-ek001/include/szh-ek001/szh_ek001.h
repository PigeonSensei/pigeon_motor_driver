#ifndef SZH_EK001_H
#define SZH_EK001_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <wiringPi.h>
#include <softPwm.h>

class Szh_ek001
{
public:
    Szh_ek001(ros::NodeHandle &n)
      : subscriber_cmd_vel_(n.subscribe("cmd_vel", 100, &Szh_ek001::CmdVelCallback, this))
       {
          // open run
          ROS_INFO("basic_class_node Open");
          GetROSParam(n);
       }
       ~Szh_ek001()
       {
          // close run
          ROS_INFO("basic_class_node Close");
       }

    int GetROSParam(ros::NodeHandle &n);

    int SetupPin();

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

  ros::Subscriber subscriber_cmd_vel_;

};

#endif // SZH_EK001_H
