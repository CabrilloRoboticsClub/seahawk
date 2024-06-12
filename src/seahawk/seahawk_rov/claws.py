"""
claws.py

Drives claws based on pilot input from the controller,

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

from seahawk_msgs.msg import ClawStates

class Claws(Node):
    """
    Class that drives the claws.
    """
    
    def __init__(self):
        """
        Initialize 'claws' node.
        """
        super().__init__("claws")

        # Clean up GPIOs incase unclean shutdown
        GPIO.cleanup()

        # Map claw names to GPIO pins
        self.claw_pins = {
            "toggle_claw":      18,
            "articulate_claw":  23,
            "back_claw":        12,
        }
        self.bool_to_mode = {True: GPIO.HIGH, False: GPIO.LOW}

        GPIO.setmode(GPIO.BCM)

        # Set claw GPIO pins to out
        GPIO.setup(self.claw_pins["toggle_claw"], GPIO.OUT)
        GPIO.setup(self.claw_pins["articulate_claw"],    GPIO.OUT)
        GPIO.setup(self.claw_pins["back_claw"],    GPIO.OUT)

        self.create_subscription(ClawStates, "claws", self.callback, 10)

    def callback(self, claw_msg: ClawStates):
        """
        Takes in input from the claw message and sets GPIO pins to values.

        Args:
            claw_msg: Message of type 'Claws' from the claws topic
        """
        GPIO.output(self.claw_pins["toggle_claw"], self.bool_to_mode[claw_msg.toggle_claw])
        GPIO.output(self.claw_pins["articulate_claw"], self.bool_to_mode[claw_msg.articulate_claw])
        GPIO.output(self.claw_pins["back_claw"], self.bool_to_mode[claw_msg.back_claw])

    def __del__(self):
        """
        "Destructor" for node. Cleans up pins when we are done with them.
        """
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = Claws()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        del node
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
