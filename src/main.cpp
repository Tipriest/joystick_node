#include "joystick.h"
#include <iostream>
#include <string>
using namespace std;

int max_value_ = (1 << 15); // 16 bits joystick
Joystick *js_;
JoystickId js_id_;

struct SimulationConfig {
  int use_joystick = 1;
  std::string joystick_type = "xbox";
  std::string joystick_device = "/dev/input/js0";
  int joystick_bits = 16;
} config;

// 初始化手柄，设置按键和轴的映射关系
void joystick_setup(std::string device, std::string js_type, int bits) {
  js_ = new Joystick(device);
  if (!js_->isFound()) {
    cout << "Error: Joystick open failed." << endl;
    exit(1);
  }

  max_value_ = (1 << (bits - 1));

  // Xbox 手柄映射
  if (js_type == "xbox") {
    cout << "Joystick Type is xbox" << endl;
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
  // Switch 手柄映射
  else if (js_type == "switch") {
    cout << "Joystick Type is xbox" << endl;
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
  } else {
    cout << "current not supported gamepad." << endl;
  }
}

int main(int argc, char **argv) {
  joystick_setup(config.joystick_device, config.joystick_type,
                 config.joystick_bits);
  while (true) {
    usleep(1000);
    JoystickEvent jsevent;
    if (js_->sample(&jsevent)) {
      if (jsevent.isButton()) {
        printf("Button %u is %s\n", jsevent.number,
               jsevent.value == 0 ? "up" : "down");
      } else if (jsevent.isAxis()) {
        printf("Axis %u is at position %d\n", jsevent.number, jsevent.value);
      }
    }
  }

  std::cout << "yes, this is good" << std::endl;
}