#include "szh-ek001/szh_ek001.h"


#include <termios.h>
#include <stdio.h>
#include <unistd.h>

int Szh_ek001::SetupPin()
{
  if(wiringPiSetupGpio() == -1)
  {
    ROS_INFO("Fail Setup WiringPI");
    return -1;
  }

  pinMode(pin_ENA_, OUTPUT);
  pinMode(pin_ENB_, OUTPUT);

  pinMode(pin_IN1_, OUTPUT);
  pinMode(pin_IN2_, OUTPUT);

  pinMode(pin_IN3_, OUTPUT);
  pinMode(pin_IN4_, OUTPUT);

  softPwmCreate(pin_ENA_, 0, 200);
  softPwmCreate(pin_ENB_, 0, 200);

  ROS_INFO("Success Setup WiringPI");
  return 0;
}

int Szh_ek001::GetROSParam(ros::NodeHandle &n)
{
  n.param<int>("IN1", pin_IN1_, 21);
  n.param<int>("IN2", pin_IN2_, 20);
  n.param<int>("IN3", pin_IN3_, 23);
  n.param<int>("IN4", pin_IN4_, 24);
  n.param<int>("ENA", pin_ENA_, 12);
  n.param<int>("ENB", pin_ENB_, 13);

  n.param<int>("min_value", min_value_, 20);

  return 0;
}

int Szh_ek001::Spin()
{
  return 0;
}

void Szh_ek001::CmdVelCallback(const geometry_msgs::Twist &cmd_vel)
{
  int value_x = round(cmd_vel.linear.x * 100);
  int value_z = round(cmd_vel.angular.z * 100);

  int rpm_L = value_x + value_z;
  int rpm_R = value_x - value_z;

//  softPwmWrite(pin_ENA_, rpm_L);
//  softPwmWrite(pin_ENB_, rpm_R);

  if(rpm_L > 0)
  {
    softPwmWrite(pin_ENA_, rpm_L + min_value_);

    digitalWrite(pin_IN2_,HIGH);
    digitalWrite(pin_IN1_,LOW);
  }
  else if(rpm_L < 0)
  {
    softPwmWrite(pin_ENA_, abs(rpm_L - min_value_));

    digitalWrite(pin_IN2_,LOW);
    digitalWrite(pin_IN1_,HIGH);
  }
  else if(rpm_L == 0)
  {
    softPwmWrite(pin_ENA_, 0);

    digitalWrite(pin_IN2_,LOW);
    digitalWrite(pin_IN1_,LOW);
  }

  if(rpm_R > 0)
  {
    softPwmWrite(pin_ENB_, rpm_R + min_value_);

    digitalWrite(pin_IN3_,HIGH);
    digitalWrite(pin_IN4_,LOW);
  }
  else if(rpm_R < 0)
  {
    softPwmWrite(pin_ENB_, abs(rpm_R - min_value_));

    digitalWrite(pin_IN3_,LOW);
    digitalWrite(pin_IN4_,HIGH);
  }
  else if(rpm_R == 0)
  {
    softPwmWrite(pin_ENB_, 0);

    digitalWrite(pin_IN3_,LOW);
    digitalWrite(pin_IN4_,LOW);
  }

//  std::cout << value << std::endl;

/*  if(value > 0)
  {
    softPwmWrite(pin_ENA_, value_x + min_value_ + value_z);
    softPwmWrite(pin_ENB_, value_x + min_value_ - value_z);
  }
  if(value == 0)
  {
    softPwmWrite(pin_ENA_, 0);
    softPwmWrite(pin_ENB_, 0);
  }
*/

//  digitalWrite(pin_IN2_,HIGH);
//  digitalWrite(pin_IN1_,LOW);
//  digitalWrite(pin_IN3_,HIGH);
//  digitalWrite(pin_IN4_,LOW);

}

int ReturnInputKey()
{
  struct termios org_term;

  char input_key = 0;

  tcgetattr(STDIN_FILENO, &org_term);

  struct termios new_term = org_term;

  new_term.c_lflag &= ~(ECHO | ICANON);

  new_term.c_cc[VMIN] = 0;
  new_term.c_cc[VTIME] = 0;

  tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

  read(STDIN_FILENO, &input_key, 1);

  tcsetattr(STDIN_FILENO, TCSANOW, &org_term);

  return input_key;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "szh_ek001_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(60);

  Szh_ek001 szh_ek001(n);

  szh_ek001.SetupPin();

  while (ros::ok())
  {
    szh_ek001.Spin();

    ros::spinOnce();
    loop_rate.sleep();
    if(ReturnInputKey() == 27) break;
  }

  digitalWrite(20,LOW);
  digitalWrite(21,LOW);

  digitalWrite(23,LOW);
  digitalWrite(24,LOW);

  return 0;

}
