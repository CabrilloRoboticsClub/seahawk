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

from seahawk_rov.i2c_sensors.bno085 import BNO085
from seahawk_rov.i2c_sensors.bme280 import BME280

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
        

        # PRESSURE SENSOR
        # TODO: Migrate all this code to its own class
        # self.pressure_publisher_ = self.create_publisher(PressureSensor, 'pressure_topic', 10)  # create a publisher that publishes messages of type String to pressure_topic

        # sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_30BA, i2c_bus=i2c) # Specify model and bus

        # sensor.init()  # initialize the sensor

        # if not sensor.init():
        #     print("SENSOR READ FAILED.")
        #     exit(1)

        # sensor.read(ms5837.OSR_256)  # Read the sensor and update the pressure and temperature.

        # if not sensor.read():
        #     print("SENSOR READ FAILED.")
        #     exit(1)
        
        # water_density = ms5837.DENSITY_FRESHWATER  # set value for the density of fresh water
        # sensor.setFluidDensity(water_density)  # Set fluid density 997 kg/m^3
        # pascal = ms5837.UNITS_Pa

        self.create_timer(0.5, self.pub_callback)
    
    def pub_callback(self):
        self.bno085.pub_callback()
        self.bme280.pub_callback()

    # def pressure_callback(self):
    #     pressure_msg = PressureSensor()  # create a obj of type Pressure
    #     pressure_msg.pressure = sensor.pressure(pascal)

    #     pressure_msg.depth sensor.depth()
    #     self.pressure_publisher_.publish(pressure_msg)  # publish depth_msg to depth_topic


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(I2C())
    # NOTE: Consider using MultiThreadedExecutor() and MutuallyExclusiveCallbackGroup()s 
    # if all sensors reading at the same speed is problematic 
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
