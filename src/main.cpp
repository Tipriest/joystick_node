#include "main.hpp"

#include <iostream>
#include <map>

using namespace std;

struct SimulationConfig {
  int use_joystick = 1;
  std::string joystick_type = "xbox";
  std::string joystick_device = "/dev/input/js0";
  int joystick_bits = 16;
} config;

typedef union {
  struct {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value;
} xKeySwitchUnion;

xKeySwitchUnion unitree_key;
std::map<string, int> AxisId = {
    {"LX", 0}, // Left stick axis x
    {"LY", 1}, // Left stick axis y
    {"RX", 3}, // Right stick axis x
    {"RY", 4}, // Right stick axis y
    {"LT", 2}, // Left trigger
    {"RT", 5}, // Right trigger
    {"DX", 6}, // Directional pad x
    {"DY", 7}, // Directional pad y
};

std::map<string, int> ButtonId = {
    {"X", 2},      //
    {"Y", 3},      //
    {"B", 1},      //
    {"A", 0},      //
    {"LB", 4},     //
    {"RB", 5},     //
    {"SELECT", 6}, //
    {"START", 7},  //
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "joystick_node");
  ros::NodeHandle nh("~");
  ros::Rate rate(100);
  joystick_setup(config.joystick_device, config.joystick_type,
                 config.joystick_bits);
  joystick_pub = nh.advertise<sensor_msgs::Joy>("joystick_msg", 1);
  sensor_msgs::Joy joystick_rosmsgs;
  joystick_rosmsgs.buttons = vector<int>(js_id_.axis.size(), 0);
  joystick_rosmsgs.axes = vector<float>(js_id_.button.size(), 0);
  vector<int> jsevent_axis_msgs(js_id_.axis.size(), 0);

  vector<int> jsevent_button_msgs(js_id_.button.size(), 0);

  while (true) {
    JoystickEvent jsevent;
    if (js_->sample(&jsevent)) {
      if (jsevent.isButton()) {
        jsevent_button_msgs[jsevent.number] = jsevent.value == 0 ? 1 : 0;
        // printf("Button %u is %s\n", jsevent.number,
        //        jsevent.value == 0 ? "up" : "down");
      } else if (jsevent.isAxis()) {
        jsevent_axis_msgs[jsevent.number] = jsevent.value;
        // printf("Axis %u is at position %d\n", jsevent.number, jsevent.value);
      }
    }
    joystick_rosmsgs.axes[0] = jsevent_axis_msgs[js_id_.axis["LX"]];
    joystick_rosmsgs.axes[1] = jsevent_axis_msgs[js_id_.axis["LY"]];
    joystick_rosmsgs.axes[2] = jsevent_axis_msgs[js_id_.axis["RX"]];
    joystick_rosmsgs.axes[3] = jsevent_axis_msgs[js_id_.axis["RY"]];
    joystick_rosmsgs.axes[4] = jsevent_axis_msgs[js_id_.axis["LT"]];
    joystick_rosmsgs.axes[5] = jsevent_axis_msgs[js_id_.axis["RT"]];
    joystick_rosmsgs.axes[6] = jsevent_axis_msgs[js_id_.axis["DX"]];
    joystick_rosmsgs.axes[7] = jsevent_axis_msgs[js_id_.axis["DY"]];

    joystick_rosmsgs.buttons[0] = jsevent_axis_msgs[js_id_.button["X"]];
    joystick_rosmsgs.buttons[1] = jsevent_axis_msgs[js_id_.button["Y"]];
    joystick_rosmsgs.buttons[2] = jsevent_axis_msgs[js_id_.button["B"]];
    joystick_rosmsgs.buttons[3] = jsevent_axis_msgs[js_id_.button["A"]];
    joystick_rosmsgs.buttons[4] = jsevent_axis_msgs[js_id_.button["LB"]];
    joystick_rosmsgs.buttons[5] = jsevent_axis_msgs[js_id_.button["RB"]];
    joystick_rosmsgs.buttons[6] = jsevent_axis_msgs[js_id_.button["SELECT"]];
    joystick_rosmsgs.buttons[7] = jsevent_axis_msgs[js_id_.button["START"]];
    joystick_pub.publish(joystick_rosmsgs);
    ros::spinOnce();
    rate.sleep();
  }

  while (false) {
    js_->getState();

    unitree_key.components.R1 = js_->button_[ButtonId["RB"]];
    unitree_key.components.L1 = js_->button_[ButtonId["LB"]];
    unitree_key.components.start = js_->button_[ButtonId["START"]];
    unitree_key.components.select = js_->button_[ButtonId["SELECT"]];
    unitree_key.components.R2 = (js_->axis_[AxisId["RT"]] > 0);
    unitree_key.components.L2 = (js_->axis_[AxisId["LT"]] > 0);
    unitree_key.components.F1 = 0;
    unitree_key.components.F2 = 0;
    unitree_key.components.A = js_->button_[ButtonId["A"]];
    unitree_key.components.B = js_->button_[ButtonId["B"]];
    unitree_key.components.X = js_->button_[ButtonId["X"]];
    unitree_key.components.Y = js_->button_[ButtonId["Y"]];
    unitree_key.components.up = (js_->axis_[AxisId["DY"]] < 0);
    unitree_key.components.right = (js_->axis_[AxisId["DX"]] > 0);
    unitree_key.components.down = (js_->axis_[AxisId["DY"]] > 0);
    unitree_key.components.left = (js_->axis_[AxisId["DX"]] < 0);

    std::cout << unitree_key.value << std::endl;

    // Restrict rate
    usleep(10000);
  }

  std::cout << "yes, this is good" << std::endl;
}