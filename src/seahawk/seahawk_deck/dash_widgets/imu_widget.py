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

        
