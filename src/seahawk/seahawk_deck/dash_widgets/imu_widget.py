from PyQt5 import QtWidgets as qtw
from PyQt5.QtGui import QPainter, QPen, QVector3D, QImage, QColor
from PyQt5.QtCore import Qt
import os

class PaintWidget(qtw.QWidget):

    def __init__(self):
        super().__init__()
        self.scale_value = 30
        self.vector = QVector3D(0, 0, 0)  # set dummy values

        # TODO: these should be in the dash_styling directory then added to the color_palete.py dict. See file for what i mean
        # Also svgs work better, I can get you some if needed

        # Commented cause I was too lazy to move these, its 2 am 
        # self.orange_up = os.path.join(os.path.dirname(__file__), 'arrow_orange_transparent_up.png')
        # self.orange_down = os.path.join(os.path.dirname(__file__), 'arrow_orange_transparent_down.png')

        # self.empty_up = os.path.join(os.path.dirname(__file__), 'arrow_empty_transparent_up.png')
        # self.empty_down = os.path.join(os.path.dirname(__file__), 'arrow_empty_transparent_down.png')


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
        pen = QPen(Qt.white, 3)
        painter.setPen(pen)
        
        # self.q_orange_up = QImage(self.orange_up)
        # self.q_orange_down = QImage(self.orange_down)

        # self.q_empty_up = QImage(self.empty_up)
        # self.q_empty_down = QImage(self.empty_down)

        # self.q_orange_up = QImage(self.q_orange_up.scaled(self.scale_value, self.scale_value))
        # self.q_orange_down = QImage(self.q_orange_down.scaled(self.scale_value, self.scale_value))

        # self.q_empty_up = QImage(self.q_empty_up.scaled(self.scale_value, self.scale_value))
        # self.q_empty_down = QImage(self.q_empty_down.scaled(self.scale_value, self.scale_value))
        
        painter.drawLine(0, 0, int(self.vector.x()), int(self.vector.y()))

        # TODO: use colors dict values instead of qt defaults
        pen = QPen(Qt.white, 7)
        painter.setPen(pen)
        painter.drawPoint(int(self.vector.x()), int(self.vector.y()))

        pen = QPen(Qt.red, 10)
        painter.setPen(pen)
        painter.drawPoint(0, 0)

        pen = QPen(Qt.red, 3)
        painter.setPen(pen)
        painter.drawLine(0, int(self.height()), 0, -int(self.height()))
        painter.drawLine(int(self.height()/2), 0, -int(self.height()/2), 0)
        painter.drawEllipse(-int(self.height()/2), -int(self.height()/2), self.height(), self.height())

        # TODO: Good, but images should again be coming from colors dict
        # if int(self.vector.z()) < 0:
        #     painter.drawImage(55, 20, self.q_empty_up)

        #     painter.drawImage(55, -20, self.q_orange_down)
        # elif int(self.vector.z()) > 0:
        #     painter.drawImage(55, 20, self.q_orange_up)

        #     painter.drawImage(55, -20, self.q_empty_down)
        # else:
        #     painter.drawImage(55, 20, self.q_empty_up)

        #     painter.drawImage(55, -20, self.q_empty_down)

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

        self._init_ui()

        # Apply css styling
        self.set_colors(colors)

        # TODO: load a style sheet (should be a param to init), see other widgets for example

    def _init_ui(self):

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

        self.paint_widget = PaintWidget()
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

