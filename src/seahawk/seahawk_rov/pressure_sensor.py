import rclpy
from rclpy import Node

import ms5837
from ms5837 import MS5837_02BA(bus=1)

class PressureReading (Node):
    def __init__(self):
        super().__init__('pressure_publisher')  # what?
        self.pressure_publisher_ = self.create_publisher(Pressure, 'pressure_topic', 10)  # create a publisher that publishes messages of type String to pressure_topic
        self.depth_publisher_ = self.create_publisher(Depth, 'depth_topic', 10)  # create a publisher that publishes messages of type Depth

        sensor = ms5837.MS5837()  # might need to specify model?
        sensor.init()  # initialize the sensor
        water_density = ms5837.DENSITY_FRESHWATER  # set value for the density of fresh water
        g = 9.81  # gravitational acceleration in m/s^2
        sensor.setFluidDensity(water_density)  # Set fluid density 997 kg/m^3
        pascal = ms5837.UNITS_Pa

        timer_period = 0.5  # space messages out by 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)  # create a timer for when the messages are published to topic

    def timer_callback(self):
        pressure_msg = Pressure()  # create a obj of type Pressure
        sensor.read(ms5837.OSR_256)  # Read the sensor and update the pressure and temperature.
        pressure_data = sensor.pressure(pascal)
        pressure_msg.data = pressure_data  # Get pressure data and give it to msg
        self.pressure_publisher_.publish(pressure_msg)  # publish pressure_msg to pressure_topic

        depth_msg = Depth()  # create object of type Depth
        depth_data  = sensor.depth()
        # depth_data = pressure_data / (water_density * g)  # getting depth using formula d = p/(roh * g)
        depth_msg.data = depth_data  # set the depth_msgs data to the calculated depth
        self.depth_publisher_.publish(depth_msg)  # publish depth_msg to depth_topic



def main(args=None):
    rclpy.init(args=args)  # what is this
    pressure_obj = PressureReading()  # create object publisher of type PressureReading
    rclpy.spin(pressure_obj)  # keep the publisher object running

    pressure_obj.destroy_node()  # terminate program when not in use
    rclpy.shutdown()  # shut down the node

if __name__ == '__main__':  # what is this
    main()  # call main?

