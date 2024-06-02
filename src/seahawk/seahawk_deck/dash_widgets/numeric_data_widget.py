from PyQt5 import QtWidgets as qtw


class NumericDataWidget(qtw.QWidget):
    """
    Creates a `NumericDataWidget` which inherits from the `qtw.QWidget` class. A `NumericDataWidget`
    displays numeric data with a title.
    """

    def __init__(self, parent: qtw.QWidget, title: str, style_sheet_file: str, colors: dict):
        """
        Initialize numeric display widget.
        
        Args:
            parent: Widget to overlay 'NumericDataWidget' on.
            title: Text title to display on widget.
            style_sheet_file: Style sheet text file formatted as a CSS f-string.
            colors: Hex codes to color widget with.
        """
        super().__init__(parent)

        with open(style_sheet_file) as style_sheet:
            self.style_sheet = style_sheet.read()

        # Define layout of frame on parent
        layout_outer = qtw.QVBoxLayout(self)
        self.setLayout(layout_outer)

        # Create frame widget
        frame = qtw.QFrame()
        layout_outer.addWidget(frame)

        # Set layout of labels on frame to grid
        layout_inner = qtw.QVBoxLayout(frame)
        frame.setLayout(layout_inner)

        # Create label widgets
        self.header = qtw.QLabel()
        self.numeric_data = qtw.QLabel()

        # Set text on widget 
        self.header.setText(title)
        self.numeric_data.setText("n/a")

        # Set an accessible name for each 
        self.header.setAccessibleName("name")
        self.numeric_data.setAccessibleName("data")
        layout_inner.addWidget(self.header)
        layout_inner.addWidget(self.numeric_data)

        self.set_colors(colors)

    def update(self, data: float):
        """
        Update data displayed by widget.

        Args:
            data: New data to display.
        """
        self.numeric_data.setText(f"{data:.2f}")

    def set_colors(self, new_colors: dict):
        """
        Sets widget colors given a dictionary of hex color codes.

        Args:
            new_colors: Hex codes to color widget with.
        """
        self.setStyleSheet(self.style_sheet.format(**new_colors)) 
