"""
servo.py

Servoooooooooooooooooooooooooooooooooooooo

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
    Class which spins the spinny thing (dpad left & right) & tilty thing (dpad up & down)
    """
    
    def __init__(self):
        """
        Initialize `servo` mega node.
        """
        super().__init__("servos")

        self.PIN_SPINNY = 19
        self.CLOCKWISE = 3
        self.COUNTERCLOCKWISE = 10 

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PIN_SPINNY, GPIO.OUT)
        self.pwm_spinny = GPIO.PWM(self.PIN_SPINNY, 50)
        self.pwm_spinny.start(0)

        self.PIN_TILTY = 13

        GPIO.setup(self.PIN_TILTY, GPIO.OUT)
        self.pwm_tilty = GPIO.PWM(self.PIN_TILTY, 50)  # is 50hZ good?
        self.pwm_tilty.start(0)

        self.create_subscription(Joy, "joy", self.callback, 10)

    def callback(self, msg: Joy):
        """
        Callback for every time the Joy message publishes.
        Sends pwm_spinny to the spinny thing 
        Sends pwm_tilty to the tilty thing
        """

        dpad = -int(msg.axes[6])  # dpad settings are left and right for the spinny thing
        if dpad == 1:
            self.pwm_spinny.ChangeDutyCycle(self.CLOCKWISE)
        elif dpad == -1:
            self.pwm_spinny.ChangeDutyCycle(self.COUNTERCLOCKWISE)
        else:
            self.pwm_spinny.ChangeDutyCycle(0)


        dpad = -int(msg.axes[7])  # dpad settings are up and down for the tilty thing
        if dpad == 1:
            # tilt up
        elif dpad == -1:
            # tilt down
        else:
            # stay the same

    def __del__(self):
        """
        "Destructor" for node. Cleans up pins when we are done with them.
        """
        self.pwm_spinny.stop()
        self.pwm_tilty.stop()
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