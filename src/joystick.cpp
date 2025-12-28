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

#include "joystick.h"
#include <cerrno>   // added
#include <unistd.h> // added (read/close)

Joystick::Joystick() { openPath("/dev/input/js0"); }

Joystick::Joystick(int joystickNumber) {
  std::stringstream sstm;
  sstm << "/dev/input/js" << joystickNumber;
  openPath(sstm.str());
}

Joystick::Joystick(std::string devicePath) { openPath(devicePath); }

Joystick::Joystick(std::string devicePath, bool blocking) {
  openPath(devicePath, blocking);
}

void Joystick::openPath(std::string devicePath, bool blocking) {
  // Allow re-opening safely
  if (_fd >= 0) {
    close(_fd);
    _fd = -1;
  }

  // Open the device using either blocking or non-blocking
  int flags = O_RDONLY | O_CLOEXEC;
  if (!blocking) flags |= O_NONBLOCK;

  _fd = open(devicePath.c_str(), flags);
}

bool Joystick::sample(JoystickEvent *event) {
  if (_fd < 0) return false;

  ssize_t bytes = 0;
  do {
    bytes = read(_fd, event, sizeof(*event));
  } while (bytes < 0 && errno == EINTR);

  if (bytes < 0) {
    // Non-blocking "no data" is not an error for sampling
    if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
    return false;
  }

  // NOTE if this condition is not met, we're probably out of sync and this
  // Joystick instance is likely unusable
  return static_cast<size_t>(bytes) == sizeof(*event);
}

bool Joystick::isFound() { return _fd >= 0; }

void Joystick::getState() {
  if (sample(&event_)) {
    if (event_.isButton()) {
      button_[event_.number] = event_.value;
    } else if (event_.isAxis()) {
      axis_[event_.number] = event_.value;
    }
  }
}

Joystick::~Joystick() {
  if (_fd >= 0) {
    close(_fd);
    _fd = -1;
  }
}

std::ostream &operator<<(std::ostream &os, const JoystickEvent &e) {
  os << "type=" << static_cast<int>(e.type)
     << " number=" << static_cast<int>(e.number)
     << " value=" << static_cast<int>(e.value);
  return os;
}
