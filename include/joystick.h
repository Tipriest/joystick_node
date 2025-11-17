// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

#pragma once

#include "joystick_event.h"
#include "unistd.h"
#include <fcntl.h>
#include <iostream>
#include <map>

#include <string>
#include <sys/stat.h>
#include <sys/types.h>

// Defaults to xbox gamepad
// 对应xbox的通道映射
struct JoystickId {

  std::map<std::string, int> axis = {
      {"LX", 0}, // Left stick axis x
      {"LY", 1}, // Left stick axis y
      {"RX", 3}, // Right stick axis x
      {"RY", 4}, // Right stick axis y
      {"LT", 2}, // Left trigger
      {"RT", 5}, // Right trigger
      {"DX", 6}, // Directional pad x
      {"DY", 7}, // Directional pad y
  };

  std::map<std::string, int> button = {
      {"X", 2},      //
      {"Y", 3},      //
      {"B", 1},      //
      {"A", 0},      //
      {"LB", 4},     //
      {"RB", 5},     //
      {"SELECT", 6}, //
      {"START", 7},  //
  };
};

/**
 * Stream insertion function so you can do this:
 *    cout << event << endl;
 */
std::ostream &operator<<(std::ostream &os, const JoystickEvent &e);

/**
 * Represents a joystick device. Allows data to be sampled from it.
 */
class Joystick {

public:
  ~Joystick();

  /**
   * Initialises an instance for the first joystick: /dev/input/js0
   */
  Joystick();

  /**
   * Initialises an instance for the joystick with the specified,
   * zero-indexed number.
   */
  Joystick(int joystickNumber);

  /**
   * Initialises an instance for the joystick device specified.
   */
  Joystick(std::string devicePath);

  /**
   * Joystick objects cannot be copied
   */
  Joystick(Joystick const &) = delete;

  /**
   * Joystick objects can be moved
   */
  Joystick(Joystick &&) = default;

  /**
   * Initialises an instance for the joystick device specified and provide
   * the option of blocking I/O.
   */
  Joystick(std::string devicePath, bool blocking);

  /**
   * Returns true if the joystick was found and may be used, otherwise false.
   */
  bool isFound();

  /**
   * Attempts to populate the provided JoystickEvent instance with data
   * from the joystick. Returns true if data is available, otherwise false.
   */

  void getState();
  bool sample(JoystickEvent *event);
  int button_[20] = {0};
  int axis_[10] = {0};

private:
  JoystickEvent event_;

  void openPath(std::string devicePath, bool blocking = false);
  int _fd;
};
