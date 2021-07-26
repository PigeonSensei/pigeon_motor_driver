#include "l298n/l298n.h"

#include <termios.h>
#include <stdio.h>
#include <unistd.h>

int L298N::SetupPin()
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

  softPwmCreate(pin_ENA_, 0, maximum_motor_command_);
  softPwmCreate(pin_ENB_, 0, maximum_motor_command_);

  ROS_INFO("Success Setup WiringPI");

  return 0;
}

int L298N::GetROSParam()
{
  ros::NodeHandle n("~");
  n.param<int>("IN1", pin_IN1_, -1);
  n.param<int>("IN2", pin_IN2_, -1);
  n.param<int>("IN3", pin_IN3_, -1);
  n.param<int>("IN4", pin_IN4_, -1);
  n.param<int>("ENA", pin_ENA_, -1);
  n.param<int>("ENB", pin_ENB_, -1);

  n.param<int>("minimum_motor_command", minimum_motor_command_, 20);
  n.param<int>("maximum_motor_command", maximum_motor_command_, 300);

  ROS_INFO("IN1 : %d, IN2 : %d,  IN3 : %d, IN4 : %d, ENA : %d, ENB: %d",
           pin_IN1_ ,pin_IN2_, pin_IN3_, pin_IN4_, pin_ENA_, pin_ENB_);
  ROS_INFO("minimum_motor_command : %d, maximum_motor_command_ : %d",
           minimum_motor_command_, maximum_motor_command_);

  return 0;
}

bool L298N::SetMotorDriverServiceCallback(l298n::SetMotorDriver::Request &req,
                                         l298n::SetMotorDriver::Response &res)
{
  if(req.command == "EMStop")
  {
    switch_.emergency_stop = 1;
    res.result = "true";
    res.message = "EMStop!";
    ROS_INFO("Motor Driver EMStop!");
    return true;
  }
  else if(req.command == "EMStopRelease")
  {
    switch_.emergency_stop = 0;
    res.result = "true";
    res.message = "EMStop Release!";
    ROS_INFO("Motor Driver EMStop Release!");
    return true;
  }
  else
  {
    res.result = "false";
    res.message = "Command that does not exist!";
    ROS_INFO("Command that does not exist!");
    return false;
  }
}

void L298N::MotorCommandCallback(const motor_driver_msgs::MotorCommand &data)
{
  motor_command_.command_L = data.command_L;
  motor_command_.command_R = data.command_R;
}

int L298N::DoGo(int command_L, int command_R)
{
  if(command_L > 100) command_L = 100;
  else if(command_L < -100) command_L = -100;

  if(command_R > 100) command_R = 100;
  else if(command_R < -100) command_R = -100;

  double ratio_command_L = ((maximum_motor_command_ - minimum_motor_command_)/100.0) * (double)command_L;
  double ratio_command_R = ((maximum_motor_command_ - minimum_motor_command_)/100.0) * (double)command_R;

  if(command_L > 0)
  {
    command_L = ratio_command_L + minimum_motor_command_;
    softPwmWrite(pin_ENA_, command_L);

    digitalWrite(pin_IN2_,HIGH);
    digitalWrite(pin_IN1_,LOW);
  }
  else if(command_L < 0)
  {
    command_L = ratio_command_L - minimum_motor_command_;
    softPwmWrite(pin_ENA_, abs(command_L));

    digitalWrite(pin_IN2_,LOW);
    digitalWrite(pin_IN1_,HIGH);
  }
  else if(command_L == 0)
  {
    softPwmWrite(pin_ENA_, 0);

    digitalWrite(pin_IN2_,LOW);
    digitalWrite(pin_IN1_,LOW);
  }

  if(command_R > 0)
  {
    command_R = ratio_command_R + minimum_motor_command_;
    softPwmWrite(pin_ENB_, command_R);

    digitalWrite(pin_IN3_,HIGH);
    digitalWrite(pin_IN4_,LOW);
  }
  else if(command_R < 0)
  {
    command_R = ratio_command_R - minimum_motor_command_;
    softPwmWrite(pin_ENB_, abs(command_R));

    digitalWrite(pin_IN3_,LOW);
    digitalWrite(pin_IN4_,HIGH);
  }
  else if(command_R == 0)
  {
    softPwmWrite(pin_ENB_, 0);

    digitalWrite(pin_IN3_,LOW);
    digitalWrite(pin_IN4_,LOW);
  }

  return 0;

}

int L298N::DoStop()
{
  softPwmWrite(pin_ENA_, 0);
  softPwmWrite(pin_ENB_, 0);

  digitalWrite(pin_IN2_,LOW);
  digitalWrite(pin_IN1_,LOW);

  digitalWrite(pin_IN3_,LOW);
  digitalWrite(pin_IN4_,LOW);

  return 0;
}

int L298N::Spin()
{
  if(switch_.emergency_stop == 0) DoGo(motor_command_.command_L, motor_command_.command_R);
  else if(switch_.emergency_stop == 1) DoStop();
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "l298n_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(60);

  L298N l298n(n);

  while (ros::ok())
  {
    l298n.Spin();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
