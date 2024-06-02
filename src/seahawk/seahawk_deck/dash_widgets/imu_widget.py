from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Imu

class IMU_Widget(qtw.QWidget):
    """
    Creates a widget that displays the lateral acceleration of the ROV. This widget
    inherits from the 'qtw.QWidget' class.
    """

    def __init__(self, parent: qtw.QWidget, colors):
        super().__init__(parent)

        self.linear_accel_x = None
        self.linear_accel_y = None
        self.linear_accel_z = None

        self._init_ros_()
        self._init_ui()


    def _init_ui(self):
        # Create an outer layout for all widgets to mount on
        layout_outer = qtw.QVBoxLayout(self)
        self.setLayout(layout_outer)

        # Create a frame on outer layout
        self.frame = qtw.QFrame()
        layout_outer.addWidget(self.frame)

        # Mount the inner layout on the frame thats attached to the outer layout
        layout_inner = qtw.QVBoxLayout(self)
        self.frame.setLayout(layout_inner)


    def _init_ros_ (self):
        self.node = rclpy.create_node('imu_widget_node')

        self.imu_subscription = self.create_subscrition(
            Imu,
            'bno085',
            self.imu_callback,
            10
        )

    def imu_callback(self, imu_data):
        self.linear_accel_x = self.imu_data.linear_acceleration.x  # probably redundant
        self.linear_accel_y = self.imu_data.linear_acceleration.y
        self.linear_accel_z = self.imu_data.linear_acceleration.z

        self.imu_widget()

    def imu_widget():

        