"""
i2c.py

Reads and publishes data from all sensors on the pi i2c bus.

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

from seahawk_rov.i2c_sensors.bno085 import BNO085
from seahawk_rov.i2c_sensors.bme280 import BME280
from seahawk_rov.i2c_sensors.pressure import Pressure

# from seahawk_msgs.msg import PressureSensor
# import ms5837
# from ms5837 import MS5837_02BA(bus=1)

import board
import busio

class I2C(Node):
    """
    Class which handles all sensors on the Raspberry Pi I2C bus.
    """
    
    def __init__(self):
        """
        Initialize `i2c` mega node.
        """
        super().__init__("i2c")
        # Grab the i2c interface for us to use
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        self.bno085 = BNO085(self, i2c)  # IMU
        self.bme280 = BME280(self, i2c)  # Pressure, Temperature, Humidity
        self.pressure = Pressure(self, i2c)

        self.create_timer(0.5, self.pub_callback)
    
    def pub_callback(self):
        self.bno085.pub_callback()
        self.bme280.pub_callback()
        self.pressure.pub_callback()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(I2C())
    # NOTE: Consider using MultiThreadedExecutor() and MutuallyExclusiveCallbackGroup()s 
    # if all sensors reading at the same speed is problematic 
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
