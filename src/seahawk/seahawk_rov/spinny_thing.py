"""
spinny_thing.py

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


class SpinnyThing(Node):
    """
    Class which spins the spinny thing from the dpad left and right.
    """
    
    def __init__(self):
        """
        Initialize `spinny_thing` mega node.
        """
        super().__init__("spinny_thing")

        self.PIN = 19
        self.CLOCKWISE = 3
        self.COUNTERCLOCKWISE = 10 

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(self.PIN, 50)
        self.pwm.start(0)

        self.create_subscription(Joy, "joy", self.callback, 10)

    def callback(self, msg: Joy):
        """
        Callback for every time the Joy message publishes.
        Sends pwm to the spinny thing.
        """
        dpad = -int(msg.axes[6])
        if dpad == 1:
            self.pwm.ChangeDutyCycle(self.CLOCKWISE)
        elif dpad == -1:
            self.pwm.ChangeDutyCycle(self.COUNTERCLOCKWISE)
        else:
            self.pwm.ChangeDutyCycle(0)

    def __del__(self):
        """
        "Destructor" for node. Cleans up pins when we are done with them.
        """
        self.pwm.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = SpinnyThing()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        del node
        rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)