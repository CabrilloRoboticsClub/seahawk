from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg

class IMU_Widget(qtw.QWidget):
    """
    Creates a widget that displays the lateral acceleration of the ROV. This widget
    inherits from the 'qtw.QWidget' class.
    """

    def __init__(self, parent: qtw.QWidget, colors):
        """
        Initialize the IMU widget

        Args:
            parent: Widget to overlay 'IMU Widget' on
        """

        super().__init__(parent)

        # Create an outer layout for all widgets to mount on
        layout_outer = qtw.QVBoxLayout(self)
        self.setLayout(layout_outer)

        # Create a frame on outer layout
        self.frame = qtw.QFrame()
        layout_outer.addWidget(self.frame)

        # Mount the inner layout on the frame thats attached to the outer layout
        layout_inner = qtw.QVBoxLayout(self)
        self.frame.setLayout(layout_inner)

        





