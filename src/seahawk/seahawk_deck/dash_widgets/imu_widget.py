# from PyQt5 import QtWidgets as qtw
# from PyQt5.QtGui import QPainter, QPen, QVector2D
# from PyQt5 import QtGui as qtg
# from PyQt5 import QtCore

from PyQt5 import QtWidgets as qtw
from PyQt5.QtGui import QPainter, QPen, QVector3D, QImage, QColor
from PyQt5.QtCore import Qt

import os

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Imu


class PaintWidget(qtw.QWidget):

    def __init__(self):
        super().__init__()
        self.height = 110
        self.width = 110
        self.scale_value = 30
        self.vector = QVector3D(0, 0, 0) # set dummy values

        self.orange_up = os.path.join(os.path.dirname(__file__), 'arrow_orange_transparent_up.png')
        self.orange_down = os.path.join(os.path.dirname(__file__), 'arrow_orange_transparent_down.png')

        self.empty_up = os.path.join(os.path.dirname(__file__), 'arrow_empty_transparent_up.png')
        self.empty_down = os.path.join(os.path.dirname(__file__), 'arrow_empty_transparent_down.png')


    def create_vector(self, x_cord, y_cord, z_cord):
        self.vector.setX(x_cord)
        self.vector.setY(y_cord)
        self.vector.setZ(z_cord)

        self.update()

    def paintEvent (self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(self.width/2, self.height/2)
        painter.scale(1, -1)
        pen = QPen(Qt.white, 5)
        painter.setPen(pen)

        self.q_orange_up = QImage(self.orange_up)
        self.q_orange_down = QImage(self.orange_down)

        self.q_empty_up = QImage(self.empty_up)
        self.q_empty_down = QImage(self.empty_down)

        self.q_orange_up = QImage(self.q_orange_up.scaled(self.scale_value, self.scale_value))
        self.q_orange_down = QImage(self.q_orange_down.scaled(self.scale_value, self.scale_value))

        self.q_empty_up = QImage(self.q_empty_up.scaled(self.scale_value, self.scale_value))
        self.q_empty_down = QImage(self.q_empty_down.scaled(self.scale_value, self.scale_value))
        
        painter.drawLine(0, 0, int(self.vector.x()), int(self.vector.y()))

        pen = QPen(Qt.white, 12)
        painter.setPen(pen)
        painter.drawPoint(int(self.vector.x()), int(self.vector.y()))

        pen = QPen(Qt.red, 5)
        painter.setPen(pen)
        painter.drawPoint(0, 0)

        pen = QPen(Qt.red, 3)
        painter.setPen(pen)
        painter.drawLine(0, int(self.height), 0, -int(self.height))
        painter.drawLine(int(self.width), 0, -int(self.width), 0)
        painter.drawEllipse(-int(self.width/2), -int(self.height/2), self.width, self.height)

        if int(self.vector.z()) < 0:
            painter.drawImage(55, 20, self.q_empty_up)

            painter.drawImage(55, -20, self.q_orange_down)
        elif int(self.vector.z()) > 0:
            painter.drawImage(55, 20, self.q_orange_up)

            painter.drawImage(55, -20, self.q_empty_down)
        else:
            painter.drawImage(55, 20, self.q_empty_up)

            painter.drawImage(55, -20, self.q_empty_down)

        painter.end()


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

        self._init_ui()

    def _init_ui(self):
        # Create an outer layout for all widgets to mount on
        self.layout_outer = qtw.QVBoxLayout(self)
        self.setLayout(self.layout_outer)

        # Create a frame on outer layout
        self.frame = qtw.QFrame()
        self.layout_outer.addWidget(self.frame)

        # Mount the inner layout on the frame thats attached to the outer layout
        self.layout_inner = qtw.QVBoxLayout(self)
        self.frame.setLayout(self.layout_inner)

        self.paint_widget = PaintWidget()
        self.layout_inner.addWidget(self.paint_widget)


    def imu_callback(self, imu_data):
        self.linear_accel_x = 50*imu_data.linear_acceleration.x  # probably redundant
        self.linear_accel_y = 50*imu_data.linear_acceleration.y
        self.linear_accel_z = 50*imu_data.linear_acceleration.z

        self.paint_widget.create_vector(self.linear_accel_x, self.linear_accel_y, self.linear_accel_z)

        
        
        