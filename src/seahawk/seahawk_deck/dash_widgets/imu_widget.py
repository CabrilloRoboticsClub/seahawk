from PyQt5 import QtWidgets as qtw
from PyQt5.QtGui import QPainter, QPen, QVector3D, QImage, QColor
from PyQt5.QtCore import Qt
import os

class PaintWidget(qtw.QWidget):

    def __init__(self, colors):
        super().__init__()
        self.scale_value = 100
        self.vector = QVector3D(0, 0, 0)  # set dummy values

        self.orange_up = colors["UP_ORANGE"]
        self.orange_down = colors["DOWN_ORANGE"]

        self.empty_up = colors["UP_EMPTY"]
        self.empty_down = colors["DOWN_EMPTY"]

        self.coordinate_colors = colors["ACCENT"]
        self.vector_colors = colors["TEXT_EMPH"]


    def create_vector(self, x_cord, y_cord, z_cord):
        self.vector.setX(x_cord)
        self.vector.setY(y_cord)
        self.vector.setZ(z_cord)

        self.update()

    def paintEvent (self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(int(self.width()/2), int(self.height()/2))
        painter.scale(1, -1)
        self.q_vector_colors = QColor(self.vector_colors)
        self.q_coordinate_colors = QColor(self.coordinate_colors)

        pen = QPen(self.q_vector_colors, 5)
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
        

        pen = QPen(self.q_vector_colors, 9)
        painter.setPen(pen)
        painter.drawPoint(int(self.vector.x()), int(self.vector.y()))

        pen = QPen(self.q_coordinate_colors, 10)
        painter.setPen(pen)
        painter.drawPoint(0, 0)

        pen = QPen(self.q_coordinate_colors, 5)
        painter.setPen(pen)
        painter.drawLine(0, int(self.height()), 0, -int(self.height()))
        painter.drawLine(int(self.height()/2), 0, -int(self.height()/2), 0)
        painter.drawEllipse(-int(self.height()/2), -int(self.height()/2), self.height(), self.height())
    
        if int(self.vector.z()) < 0:
            painter.drawImage(-int(self.height()/2) - 125, int(self.height()/40), self.q_empty_up)

            painter.drawImage(-int(self.height()/2) - 125, -int(self.height()/4), self.q_orange_down)
        elif int(self.vector.z()) > 0:
            painter.drawImage(-int(self.height()/2) - 125, int(self.height()/40), self.q_orange_up)

            painter.drawImage(-int(self.height()/2) - 125, -int(self.height()/4), self.q_empty_down)
        else:
            painter.drawImage(-int(self.height()/2) - 125, int(self.height()/40), self.q_empty_up)

            painter.drawImage(-int(self.height()/2) - 125, -int(self.height()/4), self.q_empty_down)

        painter.end()


class ImuWidget(qtw.QWidget):
    """
    Creates a widget that displays the lateral acceleration of the ROV. This widget
    inherits from the 'qtw.QWidget' class.
    """

    def __init__(self, parent: qtw.QWidget, style_sheet_file: str, colors: dict):
        super().__init__(parent)

        with open(style_sheet_file) as style_sheet:
            self.style_sheet = style_sheet.read()

        self.linear_accel_x = None
        self.linear_accel_y = None
        self.linear_accel_z = None

        self._init_ui(colors)

        # Apply css styling
        self.set_colors(colors)

        # TODO: load a style sheet (should be a param to init), see other widgets for example

    def _init_ui(self, colors):

        # with open(style_sheet_file) as style_sheet:
        #     self.style_sheet = style_sheet.read()

        # Create an outer layout for all widgets to mount on
        self.layout_outer = qtw.QVBoxLayout(self)
        self.setLayout(self.layout_outer)
        

        # Create a frame on outer layout
        self.frame = qtw.QFrame()
        self.layout_outer.addWidget(self.frame)

        # Mount the inner layout on the frame thats attached to the outer layout
        self.layout_inner = qtw.QVBoxLayout(self)
        self.frame.setLayout(self.layout_inner)

        self.paint_widget = PaintWidget(colors)
        self.layout_inner.addWidget(self.paint_widget)

    def update(self, imu_data):
        self.linear_accel_x = 100*imu_data.linear_acceleration.x  # probably redundant
        self.linear_accel_y = 100*imu_data.linear_acceleration.y
        self.linear_accel_z = 100*imu_data.linear_acceleration.z

        self.paint_widget.create_vector(self.linear_accel_x, self.linear_accel_y, self.linear_accel_z)

    def set_colors(self, new_colors: dict):
        """
        Sets widget colors given a dictionary of hex color codes.

        Args:
            new_colors: Hex codes to color widget with.
        """
        self.setStyleSheet(self.style_sheet.format(**new_colors))

