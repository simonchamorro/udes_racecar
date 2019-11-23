#!/usr/bin/env python

import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import Int16

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

PROPULSION_MAX_VOLTAGE = 8.0

NONE_MODE = 0
OPEN_LOOP_MODE = 1
CLOSED_LOOP_MODE = 2

class Gamepad:
    def __init__(self):
        self._closed_loop_mode_button = rospy.get_param('~closed_loop_mode_button', 4) # Default: L1 button
        self._open_loop_mode_button = rospy.get_param('~open_loop_mode_button', 5)     # Default: R1 button
        self._disable_joy_axis = rospy.get_param('~disable_joy_axis1', 7)              # Default: D-pad up (X-Mode)
        self._speed_axis = rospy.get_param('~speed_axis', 1)                           # Default: Left up/down
        self._direction_axis = rospy.get_param('~direction_axis', 3)                   # Default: Right left/right
        self._analog_hysteresis = rospy.get_param('~analog_hysteresis', 0.05)
        
        self.max_vel  = rospy.get_param('~max_vel',   2.0) # m/s
        self.max_angle = rospy.get_param('~max_angle', 21)  # degrees
        self.max_angle = self.max_angle * 3.14159 / 180

        self._mode_pub = rospy.Publisher('mode', Int8, queue_size=1)
        self._cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self._joy_sub = rospy.Subscriber('joy', Joy, self._joy_cb, queue_size=1)

        self._last_buttons_state = None
        self._current_mode = NONE_MODE
        self._last_mode = self._current_mode

    def _joy_cb(self, msg):
        if self._last_buttons_state is None:
            self._last_buttons_state = msg.buttons
            return

        self._handle_buttons(msg)
        self._handle_axis(msg)
        self._last_buttons_state = msg.buttons

    def _handle_buttons(self, msg):

        self._current_mode = NONE_MODE
        if msg.buttons[self._open_loop_mode_button] == 1:
            self._current_mode = OPEN_LOOP_MODE
        elif msg.buttons[self._closed_loop_mode_button] == 1:
            self._current_mode = CLOSED_LOOP_MODE

        if self._current_mode != self._last_mode or self._current_mode == NONE_MODE:
            mode_msg = Int8()
            mode_msg.data = self._current_mode
            self._mode_pub.publish(mode_msg)
            self._last_mode = self._current_mode
            if self._current_mode == NONE_MODE:
                 cmd_vel_msg = Twist()
                 cmd_vel_msg.linear.x = 0
                 cmd_vel_msg.angular.z = 0
                 cmd_vel_msg.linear.z = -1 # emergency stop!
                 self._cmd_vel_pub.publish(cmd_vel_msg)

    def _handle_axis(self, msg):

        if msg.axes[self._disable_joy_axis] == 1 and self._current_mode == CLOSED_LOOP_MODE:
            return

        propulsion = msg.axes[self._speed_axis]
        if (abs(propulsion) < self._analog_hysteresis):
            propulsion = 0

        direction = msg.axes[self._direction_axis]
        if (abs(direction) < self._analog_hysteresis):
            direction = 0

        if self._current_mode == OPEN_LOOP_MODE:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = PROPULSION_MAX_VOLTAGE * propulsion
            cmd_vel_msg.angular.z = self.max_angle * direction
            self._cmd_vel_pub.publish(cmd_vel_msg)

        elif self._current_mode == CLOSED_LOOP_MODE:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.max_vel * propulsion
            cmd_vel_msg.angular.z = self.max_angle * direction
            self._cmd_vel_pub.publish(cmd_vel_msg)

def main():
    rospy.init_node('gamepad')
    gamepad = Gamepad()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

