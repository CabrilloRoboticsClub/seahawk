from sensor_msgs.msg import Imu

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from PyQt5 import QtCore as qtc

class imu_widget(qtw.QWidget):
    def __init__(self, parent: str, task_list_file: str, style_sheet_file: str, colors: dict):
        super().__init__(parent)

        # Create layout where widget will be mounted on parent
        outer_layout = qtw.QVBoxLayout(self)
        self.setLayout(outer_layout)

        # Add a widget to mount to the outer layout
        frame = qtw.QFrame()
        outer_layout.addWidget(frame)

        # Add a grid to the widget to add text & features
        inner_layout = qtw.QVBoxLayout(frame)
        frame.setLayout(inner_layout)

        inner_layout.addStretch()

        