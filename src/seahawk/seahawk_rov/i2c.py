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
from seahawk_msgs.msg import PressureSensor

from sensor_msgs.msg import Imu
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bme280 import basic as adafruit_bme280

import ms5837
from ms5837 import MS5837_02BA(bus=1)

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

        # PRESSURE SENSOR
        self.pressure_publisher_ = self.create_publisher(PressureSensor, 'pressure_topic', 10)  # create a publisher that publishes messages of type String to pressure_topic

        sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_30BA, i2c_bus=i2c) # Specify model and bus

        sensor.init()  # initialize the sensor

        if not sensor.init():
            print("SENSOR READ FAILED.")
            exit(1)

        sensor.read(ms5837.OSR_256)  # Read the sensor and update the pressure and temperature.

        if not sensor.read():
            print("SENSOR READ FAILED.")
            exit(1)
        
        water_density = ms5837.DENSITY_FRESHWATER  # set value for the density of fresh water
        sensor.setFluidDensity(water_density)  # Set fluid density 997 kg/m^3
        pascal = ms5837.UNITS_Pa

        timer_period = 0.5  # space messages out by 0.5 seconds
        self.timer = self.create_timer(timer_period, self.pressure_callback)  # create a timer for when call back is called

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

    def pressure_callback(self):
        pressure_msg = PressureSensor()  # create a obj of type Pressure
        pressure_msg.pressure = sensor.pressure(pascal)

        pressure_msg.depth sensor.depth()
        self.pressure_publisher_.publish(pressure_msg)  # publish depth_msg to depth_topic


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(I2C())
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
