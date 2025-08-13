#pragma once
#include <sstream>

#define JS_EVENT_BUTTON 0x01 // button pressed/released
#define JS_EVENT_AXIS 0x02   // joystick moved
#define JS_EVENT_INIT 0x80   // initial state of device

class JoystickEvent {
public:
  /** Minimum value of axes range */
  static const short MIN_AXES_VALUE = -32768;

  /** Maximum value of axes range */
  static const short MAX_AXES_VALUE = 32767;

  /**
   * The timestamp of the event, in milliseconds.
   */
  unsigned int time;

  /**
   * The value associated with this joystick event.
   * For buttons this will be either 1 (down) or 0 (up).
   * For axes, this will range between MIN_AXES_VALUE and MAX_AXES_VALUE.
   */
  short value;

  /**
   * The event type.
   */
  unsigned char type;

  /**
   * The axis/button number.
   */
  unsigned char number;

  /**
   * Returns true if this event is the result of a button press.
   */
  bool isButton() { return (type & JS_EVENT_BUTTON) != 0; }

  /**
   * Returns true if this event is the result of an axis movement.
   */
  bool isAxis() { return (type & JS_EVENT_AXIS) != 0; }

  /**
   * Returns true if this event is part of the initial state obtained when
   * the joystick is first connected to.
   */
  bool isInitialState() { return (type & JS_EVENT_INIT) != 0; }

  /**
   * The ostream inserter needs to be a friend so it can access the
   * internal data structures.
   */
  friend std::ostream &operator<<(std::ostream &os, const JoystickEvent &e);
};
