from PyQt5 import QtWidgets as qtw


class TurnBankIndicator(qtw.QWidget):
    """
    TODO: Write a description of what the widget does.
    """

    def __init__(self, parent: qtw.QWidget, style_sheet_file: str, colors: dict):
        """
        Initialize turn bank indicator widget.
        
        Args:
            parent: Widget to overlay 'TurnBankIndicator' on.
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

        # Apply css styling
        self.set_colors(colors)

    def set_colors(self, new_colors: dict):
        """
        Sets widget colors given a dictionary of hex color codes.

        Args:
            new_colors: Hex codes to color widget with.
        """
        self.setStyleSheet(self.style_sheet.format(**new_colors)) 
