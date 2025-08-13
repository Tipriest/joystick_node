#include "joystick.h"
#include <iostream>
#include <map>
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
  joystick_setup(config.joystick_device, config.joystick_type,
                 config.joystick_bits);
  while (false) {
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

  while (true) {
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