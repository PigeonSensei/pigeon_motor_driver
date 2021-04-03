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

  pinMode(pin_ENA, OUTPUT);
  pinMode(pin_ENB, OUTPUT);

  pinMode(pin_IN1, OUTPUT);
  pinMode(pin_IN2, OUTPUT);

  pinMode(pin_IN3, OUTPUT);
  pinMode(pin_IN4, OUTPUT);

  softPwmCreate(pin_ENA, 0, 200);
  softPwmCreate(pin_ENB, 0, 200);

  ROS_INFO("Success Setup WiringPI");
  return 0;
}

int Szh_ek001::GetROSParam(ros::NodeHandle &n)
{
  n.param<int>("IN1", pin_IN1 ,21);
  n.param<int>("IN2", pin_IN2 ,20);
  n.param<int>("IN3", pin_IN3 ,23);
  n.param<int>("IN4", pin_IN4 ,24);
  n.param<int>("ENA", pin_ENA ,12);
  n.param<int>("ENB", pin_ENB ,13);

  return 0;
}

int Szh_ek001::Spin()
{
  return 0;
}

void Szh_ek001::CmdVelCallback(const geometry_msgs::Twist &cmd_vel)
{
  int value = round(cmd_vel.linear.x * 100);
  std::cout << value << std::endl;
  if(value > 0)
  {
    softPwmWrite(pin_ENA, value+100);
 //   softPwmWrite(pin_ENB, value+50);
  }
  if(value ==0)
  {
    softPwmWrite(pin_ENA, 0);
//    softPwmWrite(pin_ENB, 0);
  }

  digitalWrite(20,HIGH);
  digitalWrite(21,LOW);
  digitalWrite(23,HIGH);
  digitalWrite(24,LOW);

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
