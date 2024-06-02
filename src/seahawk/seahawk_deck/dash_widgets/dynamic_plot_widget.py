# dynamic_plot_widget.py
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc
import pyqtgraph as pg


from seahawk_deck.dash_styling.color_palette import DARK_MODE
COLOR_CONSTS = DARK_MODE

class DynamicPlotWidget(qtw.QWidget):
    """
    Creates a 'DynamicPlotWidget' which inherits from the 'qtw.QWidget' class. A 'DynamicPlotWidget'
    plots numeric data on a graph, updating with the `update_plot()` function
    """

    def __init__(self, parent: qtw.QWidget, x_label: str, y_label, style_sheet_file: str, colors: dict, x_range: tuple[float, float] = None, y_range: tuple[float, float] = None):
        """
        Initialize dynamic plot widget
        
        Args:
            parent: Widget to overlay 'DynamicPlotWidget' on
            style_sheet_file: Style sheet text file formatted as a CSS f-string
        """
        super().__init__(parent)
       
        with open(style_sheet_file) as style_sheet:
            self.style_sheet = style_sheet.read()

        self.x, self.y = [], []
        layout_outer = qtw.QVBoxLayout(self)
        self.setLayout(layout_outer)
        frame = qtw.QFrame()
        layout_outer.addWidget(frame)
        layout_inner = qtw.QVBoxLayout(frame)
        frame.setLayout(layout_inner)

        self.plot = pg.PlotWidget()

        if x_range is not None:
            self.plot.setXRange(*x_range)
        if y_range is not None:
            self.plot.setYRange(*y_range)
        
        self.x_label = x_label
        self.y_label = y_label

        self.plot.setTitle(f"{self.y_label} vs {self.x_label}", color=colors["TEXT"], size="11pt")
        self.plot.setBackground((0, 0, 0, 0))
        styles = {"color": colors["TEXT"], "font-size": "8pt"}
        self.plot.setLabel("left", self.y_label, **styles)
        self.plot.setLabel("bottom",self.x_label, **styles)
        self.pen = pg.mkPen(color=colors["ACCENT"], width=2)

        self.line = self.plot.plot(self.x, self.y, pen=self.pen)
        layout_inner.addWidget(self.plot)
  
        # Apply css styling
        self.set_colors(colors)
    
    def append(self, x, y):
        """
        Append new data to the line

        Args:
            x, y: New point to append
        """
        if len(self.x) > 80:
            self.x = self.x[1:]
            self.y = self.y[1:]
        self.x.append(x)
        self.y.append(y)

    def update(self, x, y):
        """
        Update data displayed by widget

        Args:
            x, y: New point to display
        """
        self.append(x, y)
        self.line.setData(self.x, self.y, pen=self.pen)
    
    def set_colors(self, new_colors: dict):
        """
        Sets widget colors given a dictionary of hex color codes.

        Args:
            new_colors: Hex codes to color widget with.
        """
        self.setStyleSheet(self.style_sheet.format(**new_colors))
        self.plot.setTitle(f"{self.y_label} vs {self.x_label}", color=new_colors["TEXT"], size="11pt")
        styles = {"color": new_colors["TEXT"], "font-size": "8pt"}
        self.plot.setLabel("left", self.y_label, **styles)
        self.plot.setLabel("bottom",self.x_label, **styles)
        self.plot.setBackground((0, 0, 0, 0))
