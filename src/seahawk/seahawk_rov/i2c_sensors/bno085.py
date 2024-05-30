"""
bno085.py

Reads and publishes data from the BNO085 sensor. The BNO085 is an IMU.
https://www.adafruit.com/product/4754

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

from sensor_msgs.msg import Imu
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C


class BNO085:
    """
    Class which reads and publishes data from the IMU.
    """

    def __init__(self, node, i2c, i2c_addr=0x4a):
        """
        Initialize `BNO085` object.
        """
        self.node = node
        self.publisher = node.create_publisher(Imu, "bno085", 10)
        self.bno085 = BNO08X_I2C(i2c_bus=i2c, address=i2c_addr)

        # enable raw data outputs
        self.bno085.enable_feature(adafruit_bno08x.BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
        self.bno085.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self.bno085.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)

    def pub_callback(self):
        """
        Collects and publishes sensor data from the IMU to the ROS network over the `/bno085` topic.
        """

        try:
            msg = Imu()
            msg.header.frame_id = "bno085"
            # IMU X right, Y forward, Z up
            # ROS Y left, X forward, Z up
            msg.orientation.x         = self.bno085.geomagnetic_quaternion[1]
            msg.orientation.y         = -self.bno085.geomagnetic_quaternion[0]
            msg.orientation.z         = self.bno085.geomagnetic_quaternion[2]
            msg.orientation.w         = self.bno085.geomagnetic_quaternion[3]
            msg.angular_velocity.x    = self.bno085.gyro[1]
            msg.angular_velocity.y    = -self.bno085.gyro[0]
            msg.angular_velocity.z    = self.bno085.gyro[2]
            msg.linear_acceleration.x = self.bno085.linear_acceleration[1]
            msg.linear_acceleration.y = -self.bno085.linear_acceleration[0]
            msg.linear_acceleration.z = self.bno085.linear_acceleration[2]
            self.publisher.publish(msg)
        except OSError:
            self.node.get_logger().info("Warning: IMU failed to publish (OSError)")
        except:
            self.node.get_logger().info("Warning: IMU failed to publish (other)")
