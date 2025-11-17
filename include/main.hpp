#pragma once
#include "joystick.h"
#include "msg_docs.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <string>
#include <thread>

class JoystickRosNode {
public:
  JoystickRosNode(ros::NodeHandle &nh);
  ~JoystickRosNode();
  // ros related
  ros::Publisher joystick_pub;
  sensor_msgs::Joy joystick_rosmsgs;

  // joystick basic related
  std::vector<int> jsevent_axis_msgs, jsevent_button_msgs;

  // std::thread 读取手柄消息的线程
  std::atomic<bool> thread_running; // 原子布尔，用于线程安全标志
  std::thread workerThread;

  // 初始化手柄
  void joystick_open();

  // 映射不同的手柄按键和轴的关系
  void joystick_mapping();

  void threadFunc();

  // 默认情况下的手柄的配置
  struct defaultJoyCfg {
    std::string joystick_type = "ps5";
    std::string joystick_device = "/dev/input/js0";
    int joystick_bits = 16;
  } defaultJoyCfg;

private:
  // joystick手柄底层接口
  Joystick *js_;
  // joystick手柄对不同类型手柄的映射
  JoystickId js_id_;
};