from PyQt5 import QtWidgets as qtw


class TriNumericDataWidget(qtw.QWidget):
    """
    Creates a `TriNumericDataWidget` which inherits from the `qtw.QWidget` class. A `TriNumericDataWidget`
    displays three numbers and a title
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
        layout_numbers = qtw.QHBoxLayout()

        # Create label widgets
        self.header = qtw.QLabel()
        self.numeric_data = [qtw.QLabel(), qtw.QLabel(), qtw.QLabel()]

        self.header.setText(title)
        self.header.setAccessibleName("name")
        
        layout_inner.addWidget(self.header)
        for data in self.numeric_data:
            data.setText("n/a")
            data.setAccessibleName("data")
            layout_numbers.addWidget(data)
        layout_inner.addLayout(layout_numbers)

        self.set_colors(colors)

    def update(self, data: list[int]):
        """
        Update data displayed by widget.

        Args:
            data: New data to display.
        """
        for data, new_data in zip(self.numeric_data, data):
            data.setText(f"{new_data:.2f}")

    def set_colors(self, new_colors: dict):
        """
        Sets widget colors given a dictionary of hex color codes.

        Args:
            new_colors: Hex codes to color widget with.
        """
        self.setStyleSheet(self.style_sheet.format(**new_colors)) 
