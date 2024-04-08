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
from std_msgs.msg import Float64
from machine import Pin
import onewire


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
        
        # The device is on GPIO7
        PIN_NUM = 7
        PIN = Pin(PIN_NUM, Pin.OUT)
        # Create one wire object for DS18B20 sensor
        self.DS18B20_ow = onewire.DS18B20(onewire.OneWire(PIN))
        # Scan for devices on the bus
        self.roms = self.DS18B20_ow.scan()
        # First instance of converting the temp
        self.DS18B20_ow.convert_temp()

    def pub_callback(self):
        """
        Read temperature from DS18B20 senor then publish it to 'DS18B20' topic
        """
        msg = Float64()
        for rom in self.roms:
            msg.data = self.DS18B20_ow.read_temp(rom)
        self.pub_temp.publish(msg)
        # Must execute the convert_temp() function to initiate a temperature reading,
        # then wait at least 750ms before reading the value. By putting this call at
        # the end of the pub_callback() so it waits a second before the function is 
        # called again due to the timer
        self.DS18B20_ow.convert_temp()


def main(args=None):
    rclpy.init(args=args)
    node = DS18B20()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
