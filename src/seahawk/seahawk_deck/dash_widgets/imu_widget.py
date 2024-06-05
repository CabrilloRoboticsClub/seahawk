# from PyQt5 import QtWidgets as qtw
# from PyQt5.QtGui import QPainter, QPen, QVector2D
# from PyQt5 import QtGui as qtg
# from PyQt5 import QtCore

from PyQt5 import QtWidgets as qtw
from PyQt5.QtGui import QPainter, QPen, QVector3D, QImage
from PyQt5.QtCore import Qt

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Imu


class PaintWidget(qtw.QWidget):

    def __init__(self):
        super().__init__()
        self.height = 50
        self.width = 50
        self.offset = 50
        self.vector = QVector3D(0, 0, 0) # set dummy values
        self.img1 = "arrow_empty_transparent.png"
        self.img2 = "arrow_orange_transparent.png"

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
        pen = QPen(Qt.black, 5)
        painter.setPen(pen)
        
        painter.drawLine(0, 0, int(self.vector.x()), int(self.vector.y()))

        pen = QPen(Qt.black, 12)
        painter.setPen(pen)
        painter.drawPoint(int(self.vector.x()), int(self.vector.y()))

        pen = QPen(Qt.red, 10)
        painter.setPen(pen)
        painter.drawPoint(0, 0)

        pen = QPen(Qt.red, 3)
        painter.setPen(pen)
        painter.drawLine(0, int(self.height), 0, -int(self.height))
        painter.drawLine(int(self.width), 0, -int(self.width), 0)
        painter.drawEllipse(-int(self.width/2), -int(self.height/2), self.width, self.height)

        if int(self.vector.z()) < 0:
            painter.drawImage(170, 155, self.q_img2)
            painter.rotate(180)
            painter.drawImage(170, -245, self.q_img1)
        elif int(self.vector.z()) > 0:
            painter.drawImage(170, 155, self.q_img1)
            painter.rotate(180)
            painter.drawImage(170, -245, self.q_img2)
        else:
            painter.drawImage(170, 155, self.q_img1)
            painter.rotate(180)
            painter.drawImage(170, -245, self.q_img1)

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
        self.linear_accel_x = self.imu_data.linear_acceleration.x  # probably redundant
        self.linear_accel_y = self.imu_data.linear_acceleration.y
        self.linear_accel_z = self.imu_data.linear_acceleration.z

        self.paint_widget.create_vector(self.linear_accel_x, self.linear_accel_y, self.linear_accel_z)

        
        
        