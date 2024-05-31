"""
bme280.py

Reads and publishes data from the BME280 sensor. The BME280
is an environmental sensor measuring relative humidity, barometric pressure
and ambient temperature.

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

from adafruit_bme280 import basic as adafruit_bme280
from seahawk_msgs.msg import Bme280


class BME280:
    """
    Class which reads and publishes data from the Bme280.
    """

    def __init__(self, node, i2c, i2c_addr=0x76):
        """
        Initialize `BME280` object.
        """
        self.node = node
        self.publisher = self.node.create_publisher(Bme280, "bme280", 10)
        self.bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, i2c_addr)

    def pub_callback(self):
        """
        Collects and publishes sensor data from the Bme280 to the ROS network over the `/bme280` topic.
        """
        try:
            msg = Bme280()
            msg.temperature = self.bme280.temperature
            msg.humidity = self.bme280.humidity
            msg.pressure = self.bme280.pressure
            self.publisher.publish(msg)
        except OSError as e:
            self.node.get_logger().info("Warning: BME280 failed to publish (OSError)\n", e)
        except Exception as e:
            self.node.get_logger().info("Warning: BME280 failed to publish (other)\n", e)



