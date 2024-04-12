"""
DS18B20.py

Reads data from DS18B20 temperature sensor

Copyright (C) 2023-2024 Cabrillo Robotics Club

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

import rclpy
from rclpy.node import Node
import board
from std_msgs.msg import Float64
from adafruit_onewire.bus import OneWireBus
from adafruit_ds18x20 import DS18X20

# https://learn.adafruit.com/using-ds18b20-temperature-sensor-with-circuitpython/circuitpython

class DS18B20(Node):
    """
    Class which reads temperature data from DS18B20
    """

    def __init__(self):
        """
        Initialize 'DS18B20' node
        """
        super().__init__("DS18B20")
        self.pub_temp = self.create_publisher(Float64, "DS18B20", 10)
        
        # Timer that waits one second between sensor reading/publish
        self.create_timer(1, self.pub_callback)
        
        # Device is on pin 7
        self.ow_bus = OneWireBus(board.D7)

       # Scan for sensors and grab the first one found
        self.DS18B20 = DS18X20(self.ow_bus, self.ow_bus.scan()[0])

    def pub_callback(self):
        """
        Read temperature from DS18B20 senor then publish it to 'DS18B20' topic
        """
        msg = Float64()
        msg.data = self.DS18B20.temperature
        self.pub_temp.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DS18B20()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
