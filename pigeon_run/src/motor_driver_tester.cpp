#include "ros/ros.h"

#include "pigeon_run/motor_driver_tester.h"

void Motor_driver_tester::DrawTUI()
{

  double gauge_command_L = 0.0;
  double gauge_command_R = 0.0;
  double negative_gauge_command_L = 1.0;
  double negative_gauge_command_R = 1.0;

  if(motor_command_.command_L > 0)
  {
    gauge_command_L = (double)motor_command_.command_L/100;
    negative_gauge_command_L = 1.0;
  }
  else if (motor_command_.command_L < 0) {
    negative_gauge_command_L = 1 - (-1 * (double)motor_command_.command_L/100);
  }
  else if (motor_command_.command_L == 0) {
    gauge_command_L = 0.0;
    negative_gauge_command_L = 1.0;
  }

  if(motor_command_.command_R > 0)
  {
    gauge_command_R = (double)motor_command_.command_R/100;
    negative_gauge_command_R = 1.0;
  }
  else if (motor_command_.command_R < 0) {
    negative_gauge_command_R = 1 - (-1 * (double)motor_command_.command_R/100);
  }
  else if (motor_command_.command_R == 0) {
    gauge_command_R = 0.0;
    negative_gauge_command_R = 1.0;
  }

  // -------- style 조건문 --------------
  auto style_0 = (menu_number_ == 0) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
  auto style_1 = (menu_number_ == 1) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
  auto style_2 = (menu_number_ == 2) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
  auto style_3 = (menu_number_ == 3) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
//  auto style_4 = (menu_number_ == 4) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);

  auto limit_style_0 = (gauge_command_L >= 1.0) ? (menu_number_ == 0) ? color(ftxui::Color::RedLight) | ftxui::dim : color(ftxui::Color::RedLight) : (menu_number_ == 0) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
  auto limit_style_1 = (gauge_command_R >= 1.0) ? (menu_number_ == 1) ? color(ftxui::Color::RedLight) | ftxui::dim : color(ftxui::Color::RedLight) : (menu_number_ == 1) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
//  auto limit_style_2 = (cmd_vel_pub_.linear.z >= 1.0) ? (menu_number_ == 2) ? color(ftxui::Color::RedLight) | ftxui::dim : color(ftxui::Color::RedLight) : (menu_number_ == 2) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
//  auto limit_style_3 = (cmd_vel_pub_.angular.x >= 1.0) ? (menu_number_ == 3) ? color(ftxui::Color::RedLight) | ftxui::dim : color(ftxui::Color::RedLight) : (menu_number_ == 3) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
//  auto limit_style_4 = (cmd_vel_pub_.angular.y >= 1.0) ? (menu_number_ == 4) ? color(ftxui::Color::RedLight) | ftxui::dim : color(ftxui::Color::RedLight) : (menu_number_ == 4) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);

  auto negative_limit_style_0 = (negative_gauge_command_L <= -1.0) ? (menu_number_ == 0) ? color(ftxui::Color::Red) | ftxui::dim : color(ftxui::Color::Red) : (menu_number_ == 0) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
  auto negative_limit_style_1 = (negative_gauge_command_R <= -1.0) ? (menu_number_ == 1) ? color(ftxui::Color::Red) | ftxui::dim : color(ftxui::Color::Red) : (menu_number_ == 1) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
//  auto negative_limit_style_2 = (cmd_vel_pub_.linear.z <= -1.0) ? (menu_number_ == 2) ? color(ftxui::Color::Red) | ftxui::dim : color(ftxui::Color::Red) : (menu_number_ == 2) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
//  auto negative_limit_style_3 = (cmd_vel_pub_.angular.x <= -1.0) ? (menu_number_ == 3) ? color(ftxui::Color::Red) | ftxui::dim : color(ftxui::Color::Red) : (menu_number_ == 3) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);
//  auto negative_limit_style_4 = (cmd_vel_pub_.angular.y <= -1.0) ? (menu_number_ == 4) ? color(ftxui::Color::Red) | ftxui::dim : color(ftxui::Color::Red) : (menu_number_ == 4) ? color(ftxui::Color::Default) | ftxui::dim : color(ftxui::Color::Default);

//  ftxui::
  ftxui::CheckBox checkbox[2];
  std::string reset_position;

  ftxui::Element Document =
  ftxui::vbox({
       // -------- Top panel --------------ftxui::text(L"Mute ☐") | style_2,
      ftxui::hbox({
           ftxui::hbox({
              ftxui::text(L"Motor_driver_tester") | ftxui::bold | ftxui::center, ftxui::separator(),
           }),
           ftxui::text(L"motor_command")| color(ftxui::Color::Red) | ftxui::center, ftxui::separator(),
           ftxui::text(L"ver 0.1")| ftxui::bold,
          // -------- cmd_vel Menu --------------
      }),
      ftxui::separator(),
      ftxui::hbox({
           ftxui::vbox({
              ftxui::text(L"-"),
              ftxui::text(L"-"),
              }) | ftxui::bold,ftxui::separator(),
           ftxui::vbox({
              ftxui::gauge(negative_gauge_command_L) | ftxui::inverted | negative_limit_style_0,
              ftxui::gauge(negative_gauge_command_R) | ftxui::inverted | negative_limit_style_1,
              }) | ftxui::flex, ftxui::separator(),
           ftxui::vbox({
              ftxui::text(L"command_L") | style_0 | ftxui::center,
              ftxui::text(L"command_R") | style_1 | ftxui::center,
               }) | ftxui::bold,ftxui::separator(),
           ftxui::vbox({
              ftxui::gauge(gauge_command_L) | limit_style_0,
              ftxui::gauge(gauge_command_R) | limit_style_1,
              }) | ftxui::flex, ftxui::separator(),
           ftxui::vbox({
              ftxui::text(L"+"),
              ftxui::text(L"+"),
              }),
      }) | ftxui::flex, ftxui::separator(),
      // -------- cmd_vel_info panel --------------
      ftxui::hbox({
          ftxui::vbox({
              ftxui::hbox({
                  ftxui::text(L"command_L  : "),
                  ftxui::text(ftxui::to_wstring(std::to_string(motor_command_.command_L))),
              }),
              ftxui::hbox({
                  ftxui::text(L"command_R  : "),
                  ftxui::text(ftxui::to_wstring(std::to_string(motor_command_.command_R))),
              }),
          }) | ftxui::bold, ftxui::separator(),
          ftxui::vbox({
              state_mute_ == 0 ? ftxui::text(L"Mute ☐") | style_2 : ftxui::text(L"Mute ▣") | style_2 ,
              state_join_ == 0 ? ftxui::text(L"Join ☐") | style_3 : ftxui::text(L"Join ▣") | style_3 ,
          }),
      }) | ftxui::flex,

  });
  Document = border(Document);
  auto screen = ftxui::Screen::Create(
  ftxui::Dimension::Full(),       // Width
  ftxui::Dimension::Fit(Document) // Height
  );

  Render(screen, Document);

  pigeon_terminal_.ClearTerminal();
  std::cout << reset_position << screen.ToString() << std::flush;
  reset_position = screen.ResetPosition();

  return;

}

int Motor_driver_tester::SetKey() // 키 입력 함수
{
  key_value_ =  pigeon_terminal_.ReturnInputKey();
  if(key_value_ == 119 | key_value_ == 87){ // INPUT W
    menu_number_ = menu_number_ - 1;
    if(menu_number_ < 0 ) menu_number_ = 0;
  }
  if(key_value_ == 115 | key_value_ == 83){ // INPUT S
    menu_number_ = menu_number_ + 1;
    if(menu_number_ > 3 ) menu_number_ = 3;
  }

  if(key_value_ == 100 | key_value_ == 68) InputMotorCommand(menu_number_, key_value_); //INPUT D
  if(key_value_ == 97 | key_value_ == 65) InputMotorCommand(menu_number_, key_value_); //INPUT A

  if(key_value_ == 122 | key_value_ == 90) ResetAllMotorCommand(); // INPUT Z
  if(key_value_ == 120 | key_value_ == 88) ResetAtMotorCommand(menu_number_); //INPUT X

  return 0;

}

void Motor_driver_tester::InputMotorCommand(int menu_number, int key_value) // 키 입력에 따른 대입 함수
{
//----------------- command_L -----------------//
  if(menu_number == 0)
  {
    if(key_value == 100 | key_value == 68){
      motor_command_.command_L = motor_command_.command_L + 1;
      if(motor_command_.command_L > 100){
        motor_command_.command_L = 100;
      }
    }
     if(key_value == 97 | key_value == 65 ){
      motor_command_.command_L = motor_command_.command_L - 1;
      if(motor_command_.command_L < -100){
        motor_command_.command_L = -100;
      }
    }
  }

//----------------- command_R -----------------//
  if(menu_number == 1)
  {
    if(key_value == 100 | key_value == 68){
      motor_command_.command_R = motor_command_.command_R + 1;
      if(motor_command_.command_R > 100){
        motor_command_.command_R = 100;
      }
    }
    if(key_value == 97 | key_value == 65){
      motor_command_.command_R = motor_command_.command_R - 1;
      if(motor_command_.command_R < -100){
        motor_command_.command_R = -100;
      }
    }
  }

//----------------- Mute -----------------//
  if(menu_number == 2)
  {
    if(key_value == 100 | key_value == 68) state_mute_ = 1;
    if(key_value == 97 | key_value == 65) state_mute_ = 0;
  }

//----------------- Join -----------------//
  if(menu_number == 3)
  {
    if(key_value == 100 | key_value == 68) state_join_ = 1;
    if(key_value == 97 | key_value == 65) state_join_ = 0;
  }





}

void Motor_driver_tester::ResetAtMotorCommand(int menu_number) // 현재 항목 motor_command 리셋 함수
{
//----------------- command_L -----------------//
  if(menu_number == 0) motor_command_.command_L = 0;

//----------------- command_R -----------------//
  if(menu_number == 1) motor_command_.command_R = 0;

//----------------- Mute -----------------//
  if(menu_number == 2) state_mute_ = 0;

//----------------- Join -----------------//
  if(menu_number == 3) state_join_ = 0;
}

void Motor_driver_tester::ResetAllMotorCommand() // 모든 항목 motor_command 리셋 함수
{
  for(int i=0;i<4;i++){
    ResetAtMotorCommand(i);
  }
}

int Motor_driver_tester::Publisher() // 토픽 퍼블리시 함수
{
  publisher_motor_command_.publish(motor_command_);

  return 0;

}

void Motor_driver_tester::Spin() // 전체 흐름 제어 함수
{
  DrawTUI();
  SetKey();
  Publisher();
//  pigeon_terminal_.ClearTerminal();
  return;
}

void Motor_driver_tester::Exit() // 종료시 값 초기화 함수
{
  ResetAllMotorCommand();
  DrawTUI();
  Publisher();
  pigeon_terminal_.ClearTerminal();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_driver_tester_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);

  Motor_driver_tester motor_driver_tester(n);

  while (ros::ok())
  {
    motor_driver_tester.Spin();
    if(motor_driver_tester.key_value_ == 27)
    {
      motor_driver_tester.Exit();
      return 0;
    }

    loop_rate.sleep();
    ros::spinOnce();

  }
  return 0;

}
