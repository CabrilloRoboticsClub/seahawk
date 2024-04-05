from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from PyQt5 import QtCore as qtc


class pressure_depth(qtw.QWidget):

    def __init__(self, parent:str, style_sheet_file:str, colors: dict):

        super().__init__(parent)

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


