from os import path

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg


PATH = path.dirname(__file__)


class ThrtCrvWidget(qtw.QWidget):
    """
    Creates a 'ThrtCrvWidget' which inherits from the 'qtw.QWidget' class. A 'ThrtCrvWidget' provides 
    a visual representation of the current chosen throttle curve
    """

    def __init__(self, parent: qtw.QWidget, colors):
        """
        Initialize throttle curve widget
        
        Args:
            parent: Widget to overlay 'ThrtCrvWidget' on
        """
        super().__init__(parent)

        self.colors = colors

        # Define layout of frame on parent
        layout_outer = qtw.QVBoxLayout(self)
        self.setLayout(layout_outer)

        # Create frame widget
        self.frame = qtw.QFrame()
        layout_outer.addWidget(self.frame)

        # Set layout of labels on frame
        layout_inner = qtw.QVBoxLayout(self.frame)
        self.frame.setLayout(layout_inner)

        self.label = qtw.QLabel()
        self.label.setScaledContents(True)
        self.label.setSizePolicy(qtw.QSizePolicy.Ignored, qtw.QSizePolicy.Ignored)
    
        self.cur_crv = None

        # Add widget to layout
        layout_inner.addWidget(self.label)
        
        # Set default curve as one
        self.update(1)

        # Apply css styling
        self.set_colors(self.colors)

    def update(self, thrt_crv: int):
        """
        Update graphical representation of the throttle curves

        Args:
            Key of throttle curve to update
        """
        # Grab the specific throttle curve img needed to be displayed on widget
        if thrt_crv == 1:
            img = self.colors["LINEAR_CURVE"]
        elif thrt_crv == 2:
            img = self.colors["CUBIC_CURVE"]
        else:
            img = self.colors["FIFTH_DEG_CURVE"]
        # Update curve image
        self.label.setPixmap(qtg.QPixmap(img))
        self.cur_crv = thrt_crv

    def set_colors(self, new_colors: dict):
        self.setStyleSheet(
            f"""
            QFrame {{
                background-color: {new_colors['SURFACE_PRIMARY']};
                border-radius: 8px;
            }}
            """
        )
        self.colors = new_colors
        self.update(self.cur_crv)
