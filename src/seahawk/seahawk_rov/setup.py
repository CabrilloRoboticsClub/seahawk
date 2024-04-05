"""
setup.py

Setup for anything on the PI, runs once upon powerup

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


class Setup(Node):
    """
    Class that drives the claws.
    """
    
    def __init__(self):
        """
        Initialize 'claws' node.
        """
        super().__init__("setup")

        # Map claw names to GPIO pins
        self.pins = {
            "fan":      27,
        }

        # GPIO.HIGH
        # GPIO.LOW

        # Set claw GPIO pins to out
        GPIO.setup(self.pins["fan"], GPIO.OUT)
        
        # Set fan to high
        GPIO.output(self.pins["fan"], GPIO.HIGH)

    def __del__(self):
        """
        "Destructor" for node. Cleans up pins when we are done with them.
        """
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = Setup()
    rclpy.spin(node)
    del node
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
