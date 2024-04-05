from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from PyQt5 import QtCore as qtc

import rclpy
from rclpy import Node

from std_msgs.msg import String

class pressure_depth(qtw.QWidget):

    def __init__(self, parent:str, style_sheet_file:str, colors: dict):
        super().__init__(parent)
        self.pressure_data = None
        self.depth_data = None

        self.init_ros()
        self.init_ui()

    def init_ros(self):
        self.node = rclpy.create_node('Pressure_widget_node')

        self.pressure_subscription = self.create_subscription(
            Pressure,'pressure_topic', self.sub_callback_pressure, 10)

        self.depth_subscription = self.create_subscription(
            Depth,'depth_topic', self.sub_callback_depth, 10)


    def init_ui(self):
        outer_layout = qtw.QVBoxLayout(self)
        self.setLayout(outer_layout)

        frame = qtw.QFrame()
        outer_layout.addWidget(frame)

        inner_layout = qtw.QVBoxLayout(frame)
        frame.setLayout(linner_layout)

        inner_layout.addStretch()

        self.pressure_title = qtw.QLabel(parent)
        self.pressure_title.setText(f"Pressure: ")

        self.depth_title  = qtw.QLabel(parent)
        self.depth_title.setText(f"Depth: ")


    def sub_callback_pressure(self, msg):
        self.pressure_data = msg.data
        self.update_pressure_ui()
    
    def sub_callback_depth(self, msg):
        self.depth_data = msg.data
        self.update_depth_ui()

    def update_pressure_ui(self):
        self.pressure_title.setText(f"Pressure: {pressure_data}")

    def update_depth_ui(self):
        self.depth_title.setText(f"Depth: {depth_data}")




