"""
seahawk_rov/main.py

this is the main node that runs on the ROV

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
from seahawk_msgs.msg import Bme280

from sensor_msgs.msg import Imu
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bme280 import basic as adafruit_bme280

import board
import busio

class I2C(Node):
    """
    Class collects data from all sensors on i2c.
    """
    
    def __init__(self):
        """
        Initialize 'claws' node.
        """
        super().__init__("i2c")
        # Grab the i2c interface for us to use
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        
        # IMU
        self.imu_pub = self.create_publisher(Imu, "imu", 10)
        self.bno085 = BNO08X_I2C(i2c_bus=i2c, address=0x4a)
        # enable raw data outputs
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)

        # BME280
        self.bme_pub = self.create_publisher(Bme280, "bme280", 10)
        self.bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c_bus=i2c, address=0x76)

    def imu_callback(self):
        msg = Imu()
        msg.header.frame_id = "bno085"

        msg.orientation.x = self.bno085.geomagnetic_quaternion[1]
        msg.orientation.y = -self.bno085.geomagnetic_quaternion[0]
        msg.orientation.z = self.bno085.geomagnetic_quaternion[2]
        msg.orientation.w = self.bno085.geomagnetic_quaternion[3]
        msg.angular_velocity.x = self.bno085.gyro[1]
        msg.angular_velocity.y = -self.bno085.gyro[0]
        msg.angular_velocity.z = self.bno085.gyro[2]
        msg.linear_acceleration.x = self.bno085.linear_acceleration[1]
        msg.linear_acceleration.y = -self.bno085.linear_acceleration[0]
        msg.linear_acceleration.z = self.bno085.linear_acceleration[2]

        # Is this the right place for this??
        try:
            self.imu_pub.publish(msg)
        except:
            self.node.get_logger().info("Warning: IMU failed to publish")
        
    def bme_callback(self):
        msg = Bme280()
        msg.temperature = self.bme280.temperature
        msg.humidity = self.bme280.humidity
        msg.pressure = self.bme280.pressure
        self.bme_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(I2C())
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
