#pragma once
#include "joystick.h"
#include "msg_docs.h"
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <string>
#include <thread>

class JoystickRosNode : public rclcpp::Node {
public:
  JoystickRosNode();
  ~JoystickRosNode();

  // ROS2 publisher
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joystick_pub;
  sensor_msgs::msg::Joy joystick_rosmsgs;

  // joystick basic related
  std::vector<int> jsevent_axis_msgs, jsevent_button_msgs;

  // std::thread for reading joystick messages
  std::atomic<bool> thread_running;
  std::thread workerThread;

  // Initialize joystick
  void joystick_open();

  // Map different joystick buttons and axes
  void joystick_mapping();

  void threadFunc();

  // Default joystick configuration
  struct DefaultJoyCfg {
    std::string joystick_type = "ps5";
    std::string joystick_device = "/dev/input/js0";
    int joystick_bits = 16;
  } defaultJoyCfg;

private:
  // Joystick底层接口
  Joystick *js_;
  // Joystick映射
  JoystickId js_id_;
};