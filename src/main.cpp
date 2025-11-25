#include "main.hpp"
#include "backward.hpp"
#include <chrono>
#include <iostream>
#include <map>

namespace backward {

backward::SignalHandling sh;

} // namespace backward

JoystickRosNode::JoystickRosNode() : Node("joystick_node") {
  // Declare and get parameters
  this->declare_parameter("joystick_type", defaultJoyCfg.joystick_type);
  this->declare_parameter("joystick_device", defaultJoyCfg.joystick_device);
  this->declare_parameter("joystick_bits", defaultJoyCfg.joystick_bits);

  this->get_parameter("joystick_type", defaultJoyCfg.joystick_type);
  this->get_parameter("joystick_device", defaultJoyCfg.joystick_device);
  this->get_parameter("joystick_bits", defaultJoyCfg.joystick_bits);

  joystick_open();
  joystick_mapping();

  // Create publisher
  joystick_pub =
      this->create_publisher<sensor_msgs::msg::Joy>("joystick_msg", 10);

  // Initialize message vectors
  joystick_rosmsgs.axes = std::vector<float>(js_id_.axis.size(), 0);
  joystick_rosmsgs.buttons = std::vector<int32_t>(js_id_.button.size(), 0);
  jsevent_axis_msgs = std::vector<int>(10, 0);
  jsevent_button_msgs = std::vector<int>(20, 0);

  thread_running = true;
  workerThread = std::thread(&JoystickRosNode::threadFunc, this);
}

JoystickRosNode::~JoystickRosNode() {
  thread_running = false;
  if (workerThread.joinable()) {
    workerThread.join();
  }
  RCLCPP_INFO(this->get_logger(),
              "joystick WorkerThread destructed, thread stopped.");
  delete js_;
}

void JoystickRosNode::joystick_open() {
  js_ = new Joystick(defaultJoyCfg.joystick_device);
  if (!js_->isFound()) {
    RCLCPP_ERROR(this->get_logger(), "Error: Joystick open failed.");
    exit(1);
  }
}

void JoystickRosNode::threadFunc() {
  while (thread_running && rclcpp::ok()) {
    JoystickEvent jsevent;
    if (js_->sample(&jsevent)) {
      if (jsevent.isButton()) {
        jsevent_button_msgs[jsevent.number] = jsevent.value == 1 ? 1 : 0;
      } else if (jsevent.isAxis()) {
        jsevent_axis_msgs[jsevent.number] = jsevent.value;
      }
    }

    joystick_rosmsgs.axes[0] = jsevent_axis_msgs[js_id_.axis["LX"]];
    joystick_rosmsgs.axes[1] = -jsevent_axis_msgs[js_id_.axis["LY"]];
    joystick_rosmsgs.axes[2] = jsevent_axis_msgs[js_id_.axis["RX"]];
    joystick_rosmsgs.axes[3] = -jsevent_axis_msgs[js_id_.axis["RY"]];
    joystick_rosmsgs.axes[4] = jsevent_axis_msgs[js_id_.axis["LT"]];
    joystick_rosmsgs.axes[5] = jsevent_axis_msgs[js_id_.axis["RT"]];
    joystick_rosmsgs.axes[6] = jsevent_axis_msgs[js_id_.axis["DX"]];
    joystick_rosmsgs.axes[7] = -jsevent_axis_msgs[js_id_.axis["DY"]];

    joystick_rosmsgs.buttons[0] = jsevent_button_msgs[js_id_.button["X"]];
    joystick_rosmsgs.buttons[1] = jsevent_button_msgs[js_id_.button["Y"]];
    joystick_rosmsgs.buttons[2] = jsevent_button_msgs[js_id_.button["B"]];
    joystick_rosmsgs.buttons[3] = jsevent_button_msgs[js_id_.button["A"]];
    joystick_rosmsgs.buttons[4] = jsevent_button_msgs[js_id_.button["LB"]];
    joystick_rosmsgs.buttons[5] = jsevent_button_msgs[js_id_.button["RB"]];
    joystick_rosmsgs.buttons[6] = jsevent_button_msgs[js_id_.button["SELECT"]];
    joystick_rosmsgs.buttons[7] = jsevent_button_msgs[js_id_.button["START"]];

    // Set timestamp
    joystick_rosmsgs.header.stamp = this->now();

    joystick_pub->publish(joystick_rosmsgs);
    usleep(2000);
  }
}

void JoystickRosNode::joystick_mapping() {
  // Xbox 手柄映射
  if (defaultJoyCfg.joystick_type == "xbox") {
    RCLCPP_INFO(this->get_logger(), "Joystick Type is xbox");
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
  else if (defaultJoyCfg.joystick_type == "beitong_bd4a") {
    RCLCPP_INFO(this->get_logger(), "Joystick Type is beitong_bd4a");
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
  }
  // ps5 手柄映射
  else if (defaultJoyCfg.joystick_type == "ps5") {
    RCLCPP_INFO(this->get_logger(), "Joystick Type is ps5");
    js_id_.axis["LX"] = 0; // 左摇杆X
    js_id_.axis["LY"] = 1; // 左摇杆Y
    js_id_.axis["RX"] = 3; // 右摇杆X
    js_id_.axis["RY"] = 4; // 右摇杆Y
    js_id_.axis["LT"] = 2; // 左扳机
    js_id_.axis["RT"] = 5; // 右扳机
    js_id_.axis["DX"] = 6; // 十字键X
    js_id_.axis["DY"] = 7; // 十字键Y

    js_id_.button["X"] = 3;
    js_id_.button["Y"] = 2;
    js_id_.button["B"] = 1;
    js_id_.button["A"] = 0;
    js_id_.button["LB"] = 4;
    js_id_.button["RB"] = 5;
    js_id_.button["SELECT"] = 8;
    js_id_.button["START"] = 9;
  } else {
    RCLCPP_WARN(this->get_logger(), "Joystick Type: %s is not supported",
                defaultJoyCfg.joystick_type.c_str());
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoystickRosNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}