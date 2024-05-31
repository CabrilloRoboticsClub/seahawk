"""
pressure.py

Reads and publishes data from the....

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

#  Pressure Sensor: https://bluerobotics.com/store/sensors-cameras/sensors/bar02-sensor-r1-rp/

from seahawk_msgs.msg import PressureSensor
import ms5837
from ms5837 import MS5837_30BA

class Pressure:

    def __init__(self, node, i2c, i2c_addr=0x76):  # This is the correct address i think
        self.node = node
        self.pressure_publisher = self.node.create_publisher(PressureSensor, 'pressure_topic', 10)  # create a publisher that publishes messages of type String to pressure_topic

        self.sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_30BA, i2c_bus=i2c) # Specify model and bus

        self.sensor.init()  # initialize the sensor
        self.sensor.read(ms5837.OSR_256)  # Read the sensor and update the pressure and temperature.)
        
        water_density = ms5837.DENSITY_FRESHWATER  # set value for the density of fresh water
        self.sensor.setFluidDensity(water_density)  # Set fluid density 997 kg/m^3
        self.pascal = ms5837.UNITS_Pa

    def pressure_callback(self):
        pressure_msg = PressureSensor()  # create a obj of type Pressure

        if self.sensor.read():
            pressure_msg = PressureSensor()  # create a obj of type Pressure
            pressure_msg.pressure = self.sensor.pressure(self.pascal)
            pressure_msg.depth = self.sensor.depth()
            self.pressure_publisher.publish(pressure_msg)  # publish depth_msg to depth_topic
        else:
            self.node.get_logger().info("Warning: ms5837 failed to publish (OSError)\n")
