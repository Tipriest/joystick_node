#include "main.hpp"
#include <iostream>
#include <map>

using namespace std;

struct SimulationConfig {
  std::string joystick_type = "ps5";
  std::string joystick_device = "/dev/input/js0";
  int joystick_bits = 16;
} config;

int main(int argc, char **argv) {
  ros::init(argc, argv, "joystick_node");
  ros::NodeHandle nh("~");
  nh.param("joystick_type", config.joystick_type, config.joystick_type);
  nh.param("joystick_device", config.joystick_device, config.joystick_device);
  nh.param("joystick_bits", config.joystick_bits, config.joystick_bits);
  ros::Rate rate(100);

  joystick_setup(config.joystick_device, config.joystick_type,
                 config.joystick_bits);
  joystick_pub = nh.advertise<sensor_msgs::Joy>("joystick_msg", 1);
  sensor_msgs::Joy joystick_rosmsgs;
  joystick_rosmsgs.axes = vector<float>(js_id_.axis.size(), 0);
  joystick_rosmsgs.buttons = vector<int>(js_id_.button.size(), 0);
  vector<int> jsevent_axis_msgs(js_id_.axis.size(), 0);

  vector<int> jsevent_button_msgs(js_id_.button.size(), 0);

  while (true) {
    JoystickEvent jsevent;
    if (js_->sample(&jsevent)) {
      if (jsevent.isButton()) {
        jsevent_button_msgs[jsevent.number] = jsevent.value == 1 ? 1 : 0;
        // printf("Button %u is %s\n", jsevent.number,
        //        jsevent.value == 0 ? "up" : "down");
      } else if (jsevent.isAxis()) {
        jsevent_axis_msgs[jsevent.number] = jsevent.value;
        // printf("Axis %u is at position %d\n", jsevent.number, jsevent.value);
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
    ros::spinOnce();
    rate.sleep();
  }
}