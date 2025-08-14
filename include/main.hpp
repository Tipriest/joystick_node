#pragma once
#include "joystick.h"
#include "msg_docs.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <string>

// values
int max_value_ = (1 << 15); // 16 bits joystick
Joystick *js_;
JoystickId js_id_;
ros::Publisher joystick_pub;

// 初始化手柄，设置按键和轴的映射关系
void joystick_setup(std::string device, std::string js_type, int bits) {
  js_ = new Joystick(device);
  if (!js_->isFound()) {
    std::cout << "Error: Joystick open failed." << std::endl;
    exit(1);
  }

  max_value_ = (1 << (bits - 1));

  // Xbox 手柄映射
  if (js_type == "xbox") {
    std::cout << "Joystick Type is xbox" << std::endl;
    js_id_.axis["LX"] = 0; // 左摇杆X
    js_id_.axis["LY"] = 1; // 左摇杆Y
    js_id_.axis["RX"] = 3; // 右摇杆X
    js_id_.axis["RY"] = 4; // 右摇杆Y
    js_id_.axis["LT"] = 2; // 左扳机
    js_id_.axis["RT"] = 5; // 右扳机
    js_id_.axis["DX"] = 6; // 十字键X
    js_id_.axis["DY"] = 7; // 十字键Y

    js_id_.button["X"] = 2;
    js_id_.button["Y"] = 3;
    js_id_.button["B"] = 1;
    js_id_.button["A"] = 0;
    js_id_.button["LB"] = 4;
    js_id_.button["RB"] = 5;
    js_id_.button["SELECT"] = 6;
    js_id_.button["START"] = 7;
  }
  // beitong_bd4a 手柄映射
  else if (js_type == "beitong_bd4a") {
    std::cout << "Joystick Type is beitong_bd4a" << std::endl;
    js_id_.axis["LX"] = 0; // 左摇杆X
    js_id_.axis["LY"] = 1; // 左摇杆Y
    js_id_.axis["RX"] = 2; // 右摇杆X
    js_id_.axis["RY"] = 3; // 右摇杆Y
    js_id_.axis["LT"] = 5; // 左扳机
    js_id_.axis["RT"] = 4; // 右扳机
    js_id_.axis["DX"] = 6; // 十字键X
    js_id_.axis["DY"] = 7; // 十字键Y

    js_id_.button["X"] = 3;
    js_id_.button["Y"] = 4;
    js_id_.button["B"] = 1;
    js_id_.button["A"] = 0;
    js_id_.button["LB"] = 6;
    js_id_.button["RB"] = 7;
    js_id_.button["SELECT"] = 10;
    js_id_.button["START"] = 11;
  } else {
    std::cout << "current not supported gamepad." << std::endl;
  }
}