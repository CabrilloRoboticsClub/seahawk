"""
servo.py

Spinnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn

Copyright (C) 2022-2023 Cabrillo Robotics Club

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Cabrillo Robotics Club
6500 Soquel Drive Aptos, CA 95003
cabrillorobotics@gmail.com
"""
import sys
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from sensor_msgs.msg import Joy
import time


class Servo(Node):
    """
    Class which spins the spinny thing from the spin_throttle left and right.
    """
    
    def __init__(self):
        """
        Initialize `spinny_thing` mega node.
        """
        super().__init__("spinny_thing")

        self.SPIN_PIN = 19
        self.TILT_PIN = 5
        self.CLOCKWISE = 3
        self.COUNTERCLOCKWISE = 10 

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.SPIN_PIN, GPIO.OUT)
        self.spin_pwm = GPIO.PWM(self.SPIN_PIN, 50)
        self.spin_pwm.start(0)

        GPIO.setup(self.TILT_PIN, GPIO.OUT)
        self.tilt_pwm = GPIO.PWM(self.TILT_PIN, 50)
        self.tilt_pwm.start(0)

        self.create_subscription(Joy, "joy", self.callback, 10)

    def callback(self, msg: Joy):
        """
        Callback for every time the Joy message publishes.
        Sends pwm to the spinny thing.
        """
        spin_throttle = -int(msg.axes[6])
        tilt_throttle = int(msg.axes[7])
        if spin_throttle == 1:
            self.spin_pwm.ChangeDutyCycle(self.CLOCKWISE)
        elif spin_throttle == -1:
            self.spin_pwm.ChangeDutyCycle(self.COUNTERCLOCKWISE)
        else:
            self.spin_pwm.ChangeDutyCycle(0)

        if tilt_throttle == 1:
            self.tilt_pwm.ChangeDutyCycle(self.CLOCKWISE)
        elif tilt_throttle == -1:
            self.tilt_pwm.ChangeDutyCycle(self.COUNTERCLOCKWISE)
        else:
            self.tilt_pwm.ChangeDutyCycle(0)

    def __del__(self):
        """
        "Destructor" for node. Cleans up pins when we are done with them.
        """
        self.spin_pwm.stop()
        self.tilt_pwm.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = Servo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        del node
        rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)
