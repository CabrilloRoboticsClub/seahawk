import sys
from os import environ, path
from threading import Thread

from PyQt5 import QtWidgets as qtw
# from PyQt5 import QtGui as qtg
# from PyQt5 import QtCore as qtc
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
import threading


from seahawk_deck.dash_styling.color_palette import DARK_MODE
from seahawk_deck.dash_widgets.countdown_widget import CountdownWidget
from seahawk_deck.dash_widgets.numeric_data_widget import NumericDataWidget
from seahawk_deck.dash_widgets.state_widget import StateWidget
from seahawk_deck.dash_widgets.throttle_curve_widget import ThrtCrvWidget
from seahawk_deck.dash_widgets.turn_bank_indicator_widget import TurnBankIndicator
from seahawk_msgs.msg import InputStates

# Size constants
# MAX_WIDTH   = 1862
# MAX_HEIGHT  = 1053
MAX_WIDTH   = 1000  # Temp for debugging
MAX_HEIGHT  = 600  # Temp for debugging

COLOR_CONSTS = DARK_MODE

PATH = path.dirname(__file__)

class MainWindow(qtw.QMainWindow):
    """
    Creates a 'MainWindow' which inherits from the 'qtw.QMainWindow' class. 'MainWindow'
    provides the main application window space to overlay widgets
    """

    def __init__(self):
        """
        Set up the 'MainWindow', overlay 'TabWidget's for multiple dash views, and display window
        """
        super().__init__()

        # Set up main window
        self.setWindowTitle("SeaHawk II Dashboard")
        self.setStyleSheet(f"background-color: {COLOR_CONSTS['MAIN_WIN_BKG']};")
        # self.setGeometry(0, 0, MAX_WIDTH, MAX_HEIGHT)

        # Create tabs
        self.tab_widget = TabWidget(self, PATH + "/dash_styling/tab_widget.txt")
        self.setCentralWidget(self.tab_widget)

        # Display window
        # self.show()
        self.showMaximized()


class TabWidget(qtw.QWidget):
    """
    Creates a 'TabWidget' which inherits from the 'qtw.QWidget' class. A 'TabWidget' provides 
    a tab bar and a page area that is used to display pages related to each tab. The tab bar is 
    shown above the page area. Each tab is associated with a different widget called a page. 
    Only the current page is shown in the page area; all the other pages are hidden. The user can 
    show a different page by clicking on its tab
    """

    def __init__(self, parent: MainWindow, style_sheet_file: str):
        """
        Initialize tabs

        Args:
            parent: Window where to place tabs
            tab_names: List of tab names
            style_sheet_file: Style sheet text file formatted as a CSS f-string
        """
        super().__init__(parent)
        
        # Define layout of tabs
        layout = qtw.QVBoxLayout(self)
        self.setLayout(layout)

        # Initialize tabs
        tabs = qtw.QTabWidget()

        # Create a dict in which the key is the provided name of the tab, and the value is a qtw.QWidget() object
        tab_names = ["Pilot", "Co-Pilot", "VPF", "Debug", "Cameras", "Control Mapping"]
        self.tab_dict = {name: qtw.QWidget() for name in tab_names}

        # Add tabs
        for name, tab in self.tab_dict.items():
            tabs.addTab(tab, name)
        
        # Apply css styling
        with open(style_sheet_file) as style_sheet:
            self.setStyleSheet(style_sheet.read().format(**COLOR_CONSTS))
        
        # Add tabs to widget
        layout.addWidget(tabs)

        self.create_pilot_tab(self.tab_dict["Pilot"])
    
    def create_pilot_tab(self, tab):
        """
        Creates pilot dash tab with the following widgets:
            - Feature states:   Displays the states of Bambi Mode (on/off), the claw (closed/open), CoM shift (engaged/not)
            - Throttle curve:   Displays the activated throttle curve
            - Temperature:      Displays the temperature reading
            - Depth:            Displays the depth reading
            - IMU:              Displays the IMU readings as a turn/bank indicator (graphic to help keep constant acceleration)
            - Countdown:        Displays a countdown
        """
        WIDGET_WIDTH = 180
        WIDGET_HEIGHT = 160

        # Display feature state widget
        self.state_widget = StateWidget(tab, ["Bambi Mode", "Claw", "CoM Shift"], PATH + "/dash_styling/state_widget.txt")
        self.state_widget.resize(WIDGET_WIDTH, WIDGET_HEIGHT) # FIXME: This should probably not be a fixed value
        # state_widget.update_state("Claw")

        # Display throttle curve widget
        self.thrt_crv_widget = ThrtCrvWidget(tab)
        self.thrt_crv_widget.move(0, 150)
        self.thrt_crv_widget.resize(WIDGET_WIDTH, WIDGET_HEIGHT)

        self.temp_widget = NumericDataWidget(tab, "Temperature", PATH + "/dash_styling/numeric_data_widget.txt")
        self.temp_widget.move(0, 300)
        self.temp_widget.resize(WIDGET_WIDTH, WIDGET_HEIGHT)

        self.depth_widget = NumericDataWidget(tab, "Depth", PATH + "/dash_styling/numeric_data_widget.txt")
        self.depth_widget.move(0, 450)
        self.depth_widget.resize(WIDGET_WIDTH, WIDGET_HEIGHT)
        
        # FIXME: Fix path when a updated css file gets made
        self.turn_bank_indicator_widget = TurnBankIndicator(tab, PATH + "/dash_styling/numeric_data_widget.txt")
        self.turn_bank_indicator_widget.move(0, 600)
        self.turn_bank_indicator_widget.resize(WIDGET_WIDTH, WIDGET_HEIGHT)

        self.countdown_widget = CountdownWidget(tab, PATH + "/dash_styling/countdown_widget.txt", minutes=15, seconds=0)
        self.countdown_widget.move(0, 750)
        self.countdown_widget.resize(WIDGET_WIDTH, 210)
    
    def update_pilot_tab_input_states(self, state_to_update: str):
        """
        Update gui display of input states

        Args:
            feat_state_update: List of values to update for the feature widget
            thrt_crv_update: Updated value for throttle curve
        """
        self.state_widget.update_state(state_to_update["state_widget"])
        self.thrt_crv_widget.update_thrt_crv(state_to_update["throttle_curve"])


def fix_term():
    """
    If VS Code was installed with snap, the 'GTK_PATH' variable must be unset.
    This is automated in this function
    """
    if "GTK_PATH" in environ and "snap" in environ["GTK_PATH"]:
        environ.pop("GTK_PATH")


def run_node(dash, args):
    """
    Creates and runs a ROS node which updates the PyQt dashboard from ROS topics
    """
    rclpy.init(args=args)
    dash_node = rclpy.create_node("dash")


    def callback_input_states(input_state_msg: InputStates,): 
        """
        For every message published to the 'input_states' topic, update the relevant values on the gui 

        Updates dash representation of:
            - Bambi mode
            - Claw state 
            - CoM shift
            - Throttle curve option
        Based on values from 'input_states' topic

        Args:
            input_state_msg: Message from the type 'InputStates' from the 'input_states' topic
        """
        # Map the values sent from 'input_states' to feature names
        input_state_dict = {
            "state_widget": {
                "Bambi Mode":   input_state_msg.bambi_mode,
                "Claw":         input_state_msg.claw_state,
                "CoM Shift":    input_state_msg.com_shift,
            },
            "throttle_curve":   int(input_state_msg.throttle_curve),
        }
        dash.tab_widget.update_pilot_tab_input_states(input_state_dict)


    # Create a subscription to 'input_state' and call the `callback_input_states` function for every message
    dash_node.create_subscription(InputStates, "input_states", callback_input_states, 10)
    rclpy.spin(dash_node)
    rclpy.shutdown()


def main(args=None):
    # Setup dashboards
    fix_term()
    app = qtw.QApplication([])
    pilot_dash = MainWindow()

    # Threading allows the process to display the dash and run the node at the same time
    # Create and start a thread for run_node the function which creates and runs the node
    node_thread = threading.Thread(target=run_node, args=(pilot_dash, args))
    node_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main(sys.argv)