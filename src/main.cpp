#include "main.hpp"
#include "backward.hpp"
#include <iostream>
#include <map>

namespace backward {

backward::SignalHandling sh;

} // namespace backward

JoystickRosNode::JoystickRosNode(ros::NodeHandle &nh) {
  nh.param("joystick_type", defaultJoyCfg.joystick_type,
           defaultJoyCfg.joystick_type);
  nh.param("joystick_device", defaultJoyCfg.joystick_device,
           defaultJoyCfg.joystick_device);
  nh.param("joystick_bits", defaultJoyCfg.joystick_bits,
           defaultJoyCfg.joystick_bits);
  joystick_open();
  joystick_mapping();
  joystick_pub = nh.advertise<sensor_msgs::Joy>("joystick_msg", 0);
  joystick_rosmsgs.axes = std::vector<_Float32>(js_id_.axis.size(), 0);
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
  std::cout << "joystick WorkerThread destructed, thread stopped." << std::endl;
}

void JoystickRosNode::joystick_open() {
  js_ = new Joystick(defaultJoyCfg.joystick_device);
  if (!js_->isFound()) {
    std::cout << "Error: Joystick open failed." << std::endl;
    exit(1);
  }
  return;
}

void JoystickRosNode::threadFunc() {
  while (thread_running) {
    JoystickEvent jsevent;
    if (js_->sample(&jsevent)) {
      if (jsevent.isButton()) {
        jsevent_button_msgs[jsevent.number] = jsevent.value == 1 ? 1 : 0;
        // printf("Button %u is %s\n", jsevent.number,
        //        jsevent.value == 0 ? "up" : "down");
      } else if (jsevent.isAxis()) {
        jsevent_axis_msgs[jsevent.number] = jsevent.value;
        // printf("Axis %u is at position %d\n", jsevent.number,
        // jsevent.value);
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
    joystick_pub.publish(joystick_rosmsgs);
    usleep(10000);
  }
}

void JoystickRosNode::joystick_mapping() {
  // Xbox 手柄映射
  if (defaultJoyCfg.joystick_type == "xbox") {
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
  else if (defaultJoyCfg.joystick_type == "beitong_bd4a") {
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
  }
  // ps5 手柄映射
  else if (defaultJoyCfg.joystick_type == "ps5") {
    std::cout << "Joystick Type is ps5" << std::endl;
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
    std::cout << "Joystick Type: " << defaultJoyCfg.joystick_type
              << " is not supported" << std::endl;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joystick_node");
  ros::NodeHandle nh("~");
  JoystickRosNode joynode(nh);
  ros::Rate rate(100);
  ros::spin();
  return 0;
}