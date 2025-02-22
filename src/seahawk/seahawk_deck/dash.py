import sys
from os import environ, path
from threading import Thread

from PyQt5 import QtCore as qtc
from PyQt5 import QtGui as qtg
from PyQt5 import QtWidgets as qtw
from PyQt5.QtGui import QKeyEvent
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node 
from rclpy.publisher import Publisher
from std_msgs.msg import String, Float32
from rcl_interfaces.msg import ParameterEvent
from sensor_msgs.msg import Image

from seahawk_deck.dash_styling.color_palette import DARK_MODE, LIGHT_MODE
from seahawk_deck.dash_widgets.countdown_widget import CountdownWidget
from seahawk_deck.dash_widgets.numeric_data_widget import NumericDataWidget
from seahawk_deck.dash_widgets.state_widget import StateWidget
from seahawk_deck.dash_widgets.throttle_curve_widget import ThrtCrvWidget
from seahawk_deck.dash_widgets.term_widget import TermWidget
from seahawk_deck.set_remote_params import SetRemoteParams
from seahawk_deck.dash_widgets.tri_numeric_data_widget import TriNumericDataWidget
from seahawk_deck.dash_widgets.dynamic_plot_widget import DynamicPlotWidget
from seahawk_msgs.msg import InputStates, DebugInfo, Bme280, PressureSensor

PATH = path.dirname(__file__)

DEFAULT_COLORS = DARK_MODE


class RosQtBridge(qtw.QWidget):
    """
    Thread safe bridge between ROS and Qt using signals and slots. `RosQtBridge`
    object instance must be passed to the node and main window class.
    """
    
    # Signals (must be class variables)
    new_input_state_msg_sgl = qtc.pyqtSignal()
    new_cam_down_msg_sgl = qtc.pyqtSignal()
    new_cam_back_msg_sgl = qtc.pyqtSignal()
    new_cam_front_msg_sgl = qtc.pyqtSignal()
    new_com_param_sgl = qtc.pyqtSignal()
    new_debug_sgl = qtc.pyqtSignal()
    new_bme280_sgl = qtc.pyqtSignal()
    new_temperature_sgl = qtc.pyqtSignal()
    new_publisher_sgl = qtc.pyqtSignal()
    new_set_params_sgl = qtc.pyqtSignal()
    new_pressure_msg_sgl = qtc.pyqtSignal()

    def __init__(self):
        """
        Initialize `RosQtBridge` object.
        """
        super().__init__()
        # Variables to transfer data between ROS and Qt
        self.input_state_msg = None
        self.cam_down_msg = None 
        self.cam_back_msg = None
        self.cam_front_msg = None
        self.com = [0.0] * 3
        self.debug_msg = None
        self.bme280_msg = None
        self.temperature_msg = None
        self.keystroke_pub = None
        self.pilot_input_set_params = None

    def callback_input_states(self, msg: InputStates):
        """
        Called for each time a message is published to the `input_states` topic.
        Collects the contents of the message sent and emits a `new_input_state_msg_sgl`
        signal which is received by Qt.

        Args:
            msg: Message of type `InputStates` from the `input_states` topic.
        """
        self.input_state_msg = msg
        self.new_input_state_msg_sgl.emit()

    def callback_cam_down(self, msg: Image):
        """
        Called for each time a message is published to the `camera/down/h264` topic.
        Collects the contents of the message sent and emits a `new_cam_down_msg_sgl`
        signal which is received by Qt.

        Args:
            msg: Message of type `Image` from the `camera/down/h264` topic.
        """
        self.cam_down_msg = msg
        self.new_cam_down_msg_sgl.emit()

    def callback_cam_back(self, msg: Image):
        """
        Called for each time a message is published to the `camera/back/h264` topic.
        Collects the contents of the message sent and emits a `new_cam_back_msg_sgl`
        signal which is received by Qt.

        Args:
            msg: Message of type `Image` from the `camera/back/h264` topic.
        """
        self.cam_back_msg = msg
        self.new_cam_back_msg_sgl.emit()

    def callback_cam_front(self, msg: Image):
        """
        Called for each time a message is published to the `camera/front/h264` topic.
        Collects the contents of the message sent and emits a `new_cam_front_msg_sgl`
        signal which is received by Qt.

        Args:
            msg: Message of type `Image` from the `camera/front/h264` topic
        """
        self.cam_front_msg = msg
        self.new_cam_front_msg_sgl.emit()
    
    def callback_param_event(self, msg: ParameterEvent):
        """
        Called for each time a parameter is modified on the ROS network.

        Args:
            msg: Parameter event which has occurred.
        """
        if msg.node == "/thrust":
            for param in msg.changed_parameters:
                if param.name == "center_of_mass_increment":
                    if list(param.value.double_array_value)[0:3] == [0.0] * 3:
                        self.com = [0.0] * 3
                    else:
                        for i, val in enumerate(param.value.double_array_value):
                            self.com[i] += val
                    self.new_com_param_sgl.emit()
    
    def callback_debug(self, msg: DebugInfo):
        """
        Called for each time a message is published to the `debug_info` topic.
        Collects the contents of the message sent and emits a `new_debug_sgl`
        signal which is received by Qt.

        Args:
            msg: Message of type `DebugInfo` from the `debug_info` topic
        """
        self.debug_msg = msg
        self.new_debug_sgl.emit()
    
    def callback_bme280(self, msg: Bme280):
        """
        Called for each time a message is published to the `bme280` topic.
        Collects the contents of the message sent and emits a `new_bme280_sgl`
        signal which is received by Qt.

        Args:
            msg: Message of type `Bme280` from the `bme280` topic
        """
        self.bme280_msg = msg
        self.new_bme280_sgl.emit()

    def callback_temperature(self, msg: Float32):
        self.temperature_msg = msg
        self.new_temperature_sgl.emit()

    def callback_pressure(self, msg: PressureSensor):
        """
        Gives Qt access to the pressure topic and emits 'new_pressure_msg_sgl'

        Args:
            msg: messages of type PressureSensor recieved from 'pressure_topic'
        """

        self.pressure_msg = msg
        self.new_pressure_msg_sgl.emit()

    def add_publisher(self, pub: Publisher):
        """
        Gives Qt access to a ROS publisher and emits a `new_publisher_sgl`.

        Args:
            pub: Publisher to add, publishes to `keystroke` topic.
        """
        self.keystroke_pub = pub
        self.new_publisher_sgl.emit()
    
    def add_set_params(self, pilot_input_set_params: SetRemoteParams, thrust_set_params: SetRemoteParams):
        """
        Gives Qt access to a `SetRemoteParams` and emits a `pilot_input_set_params`.

        Args:
            pilot_input_set_params: SetRemoteParams instance to give Qt access to.
            thrust_set_params: SetRemoteParams instance to give Qt access to.
        """
        self.pilot_input_set_params = pilot_input_set_params
        self.thrust_set_params = thrust_set_params
        self.new_set_params_sgl.emit()


class VideoFrame():
    """
    Data needed to display a video frame.
    """

    def __init__(self):
        """
        Set up the 'VideoFrame' attributes initial values.
        """
        self.image = None
    
        self.label = qtw.QLabel()
        # Fit video frame to size of label, no need to resize it later
        self.label.setScaledContents(True)
        self.label.setSizePolicy(qtw.QSizePolicy.Ignored, qtw.QSizePolicy.Ignored)


class MainWindow(qtw.QMainWindow):
    """
    Creates a 'MainWindow' which inherits from the 'qtw.QMainWindow' class. 'MainWindow'
    provides the main dash window space to overlay widgets.
    """

    def __init__(self, ros_qt_bridge: RosQtBridge):
        """
        Set up the 'MainWindow', overlay 'TabWidget's for multiple dash views, and display window.

        Args:
            ros_qt_bridge: Bridge object for functionality between ROS and Qt.
        """
        super().__init__()

        self.ros_qt_bridge = ros_qt_bridge
        self.ros_qt_bridge.new_com_param_sgl.connect(self.com_param_callback)
        self.ros_qt_bridge.new_publisher_sgl.connect(self.init_publisher)
        self.ros_qt_bridge.new_set_params_sgl.connect(self.add_set_params)
        self.keystroke_pub = None
        self.pilot_input_set_params = None
        self.com_set_params = None
        self.com_choice = None
        self.com_shift = [0.0, 0.0, 0.0]

        # Set up main window
        self.colors = DEFAULT_COLORS
        self.setWindowTitle("SeaHawk II Dashboard")
        self.setStyleSheet(f"background-color: {self.colors['MAIN_WIN_BKG']};")

        # Create tabs
        self.tab_widget = TabWidget(self, self.ros_qt_bridge, PATH + "/dash_styling/tab_widget.txt", self.colors)
        self.setCentralWidget(self.tab_widget)

        # Display window
        self.showMaximized()

    @qtc.pyqtSlot()
    def init_publisher(self):
        """
        Adds the keystroke publisher to `MainWindow`.
        """
        self.keystroke_pub = self.ros_qt_bridge.keystroke_pub

    @qtc.pyqtSlot()
    def add_set_params(self):
        """
        Adds the pilot input set params object to `MainWindow`.
        """
        self.pilot_input_set_params = self.ros_qt_bridge.pilot_input_set_params
        self.thrust_set_params = self.ros_qt_bridge.thrust_set_params
    
    @qtc.pyqtSlot()
    def com_param_callback(self):
        """
        Updates display of CoM widget each time the parameter is updated
        """
        # self.com_shift = self.ros_qt_bridge.com_shift
        self.tab_widget.com_shift_widget.update(self.ros_qt_bridge.com)

    def keyPressEvent(self, a0: QKeyEvent) -> None:
        """
        Called for each time there is a keystroke. Publishes the code of the key that was
        pressed or released to the ROS topic `keystroke` and sets parameters of other nodes
        dependant on keystrokes
        """
        try:
            data = str(chr(a0.key()))
        except ValueError:
            data = "Invalid key"
        
        # Publish key to 'keystroke' topic
        msg = String()
        msg.data = data
        self.keystroke_pub.publish(msg)

        # Update throttle curve parameter
        if data in {"1", "2", "3"}:
            self.pilot_input_set_params.update_params("throttle_curve_choice", int(data))
            self.pilot_input_set_params.send_params()

        # Update Com Shift
        if data in {"X", "Y", "Z"}:
            self.com_choice = data
        elif data not in {"-", "+", "=", "Invalid key"}:
            self.com_choice = None
    
        if self.com_choice and data in {"-", "+", "="}:
            increment = [0.0 if i != ord(self.com_choice) - 88 else 0.01 if data in {"+", "="} else -0.01 for i in range(3)]
            self.thrust_set_params.update_params("center_of_mass_increment", increment)
            self.thrust_set_params.send_params()
        
        if data == "C":
            self.thrust_set_params.update_params("center_of_mass_increment", [0.01, 0.0, 0.0])
            self.thrust_set_params.send_params()
        elif data == "V":
            self.thrust_set_params.update_params("center_of_mass_increment", [-0.01, 0.0, 0.0])
            self.thrust_set_params.send_params()

        # Change colors mode between light and dark mode
        if data == "0":
            if self.colors == DARK_MODE: self.update_colors(LIGHT_MODE)
            else: self.update_colors(DARK_MODE)

    def update_colors(self, new_colors: dict):
        self.colors = new_colors
        self.setStyleSheet(f"background-color: {self.colors['MAIN_WIN_BKG']};")
        self.tab_widget.set_colors(self.colors)
        self.tab_widget.state_widget.set_colors(self.colors)
        self.tab_widget.com_shift_widget.set_colors(self.colors)
        self.tab_widget.thrt_crv_widget.set_colors(self.colors)
        self.tab_widget.temperature_widget.set_colors(self.colors)
        self.tab_widget.depth_widget.set_colors(self.colors)
        self.tab_widget.countdown_widget.set_colors(self.colors)
        self.tab_widget.term_widget.set_colors(self.colors)
        self.tab_widget.leak.set_colors(self.colors)
        self.tab_widget.humidity.set_colors(self.colors)
        self.tab_widget.barometric_pressure.set_colors(self.colors)
        self.tab_widget.ambient_temperature.set_colors(self.colors)
        self.tab_widget.cpu_usage.set_colors(self.colors)
        self.tab_widget.memory_usage.set_colors(self.colors)
        self.tab_widget.cpu_temperature.set_colors(self.colors)
        self.tab_widget.net_sent.set_colors(self.colors)
        self.tab_widget.net_recv.set_colors(self.colors)


class TabWidget(qtw.QWidget):
    """
    Creates a `TabWidget` which inherits from the `qtw.QWidget` class. A `TabWidget` provides 
    a tab bar and a page area that is used to display pages related to each tab. The tab bar is 
    shown above the page area. Each tab is associated with a different widget called a page. 
    Only the current page is shown in the page area; all the other pages are hidden. The user can 
    show a different page by clicking on its tab
    """

    def __init__(self, parent: MainWindow, ros_qt_bridge: RosQtBridge, style_sheet_file: str, colors: dict):
        """
        Initialize tab widget.

        Args:
            parent: Window where to place tabs.
            ros_qt_bridge: Bridge object for functionality between ROS and Qt.
            style_sheet_file: Style sheet text file formatted as a CSS f-string.
            colors: Hex codes to color widget with.
        """
        super().__init__(parent)

        self.colors = colors
        with open(style_sheet_file) as style_sheet:
            self.style_sheet = style_sheet.read()
        
        # Bridge between ros and the rt dashboard
        self.ros_qt_bridge = ros_qt_bridge
        # Connect signals to slots for thread safe communication between ROS and Qt
        self.ros_qt_bridge.new_input_state_msg_sgl.connect(self.update_pilot_tab_input_states)
        self.ros_qt_bridge.new_cam_down_msg_sgl.connect(self.update_cam_down)
        self.ros_qt_bridge.new_cam_back_msg_sgl.connect(self.update_cam_back)
        self.ros_qt_bridge.new_cam_front_msg_sgl.connect(self.update_cam_front)
        self.ros_qt_bridge.new_debug_sgl.connect(self.update_debug)
        self.ros_qt_bridge.new_bme280_sgl.connect(self.update_bme280)
        self.ros_qt_bridge.new_temperature_sgl.connect(self.update_temperature)
        self.ros_qt_bridge.new_pressure_msg_sgl.connect(self.update_pressure_msg)
    
        # Define layout of tabs
        layout = qtw.QVBoxLayout(self)
        self.setLayout(layout)

        # Initialize tabs
        tabs = qtw.QTabWidget()
        tabs.currentChanged.connect(self.tab_changed)
        
        # Tabs open
        self.pilot_open = True
        self.debug_open = False

        # Create a dict in which the key is the provided name of the tab, and the value is a qtw.QWidget() object
        tab_names = ["Pilot", "Co-Pilot", "Debug", "Control Mapping"]
        self.tab_dict = {name: qtw.QWidget() for name in tab_names}

        # Add tabs
        for name, tab in self.tab_dict.items():
            tabs.addTab(tab, name)

        self.setStyleSheet(self.style_sheet.format(**self.colors))
        
        # Add tabs to widget
        layout.addWidget(tabs)

        # Create specific tabs
        self.create_pilot_tab(self.tab_dict["Pilot"])
        self.create_debug_tab(self.tab_dict["Debug"])

        # Apply css styling
        self.set_colors(self.colors)

    
    def set_colors(self, new_colors: dict):
        """
        Sets widget colors given a dictionary of hex color codes.

        Args:
            new_colors: Hex codes to color widget with.
        """
        self.setStyleSheet(self.style_sheet.format(**new_colors))
        self.demo_map.setPixmap(qtg.QPixmap(new_colors["MAP_IMG"]))

    def create_pilot_tab(self, tab: qtw.QWidget):
        """
        Creates pilot dash tab with the following widgets:
            - Feature states:   Displays the states of Bambi Mode, Kill Button, Reversed controls 
            - Throttle curve:   Displays the activated throttle curve
            - Temperature:      Displays the temperature reading
            - Depth:            Displays the depth reading
            - IMU:              Displays the IMU readings as a turn/bank indicator (graphic to help keep constant acceleration)
            - Countdown:        Displays a countdown
            - Down camera       Displays video feed from down camera
            - back camera       Displays video feed from back camera
            - Front camera      Displays video feed from front camera
            - Product demo map  Displays a static image of the product demo area map
        
        Args: 
            tab: Tab object instance of pilot tab.
        """
        # Setup layouts
        home_window_layout = qtw.QHBoxLayout(tab)
        vert_widgets_layout = qtw.QVBoxLayout()
        vert_widgets_layout.setSpacing(0)
        cam_layout = qtw.QGridLayout()

        # Create widgets
        self.state_widget = StateWidget(tab, ["Bambi Mode", "Kill Button", "Reversed"], PATH + "/dash_styling/state_widget.txt", self.colors)
        self.com_shift_widget = TriNumericDataWidget(tab, "CoM Shift", PATH + "/dash_styling/tri_numeric_data_widget.txt", self.colors)
        self.thrt_crv_widget = ThrtCrvWidget(tab, self.colors)
        self.temperature_widget = NumericDataWidget(tab, "Temperature (C)", PATH + "/dash_styling/numeric_data_widget.txt", self.colors)
        self.depth_widget = NumericDataWidget(tab, "Depth (m)", PATH + "/dash_styling/numeric_data_widget.txt", self.colors)
        self.pressure_sensor_widget = NumericDataWidget(tab, "Pressure", PATH + "/dash_styling/numeric_data_widget.txt", self.colors)
        self.countdown_widget = CountdownWidget(tab, PATH + "/dash_styling/countdown_widget.txt", self.colors, minutes=15, seconds=0)

        # Add widgets to side vertical layout
        # Stretch modifies the ratios of the widgets (must add up to 100)
        vert_widgets_layout.addWidget(self.state_widget, stretch=18)
        vert_widgets_layout.addWidget(self.com_shift_widget, stretch=8)
        vert_widgets_layout.addWidget(self.thrt_crv_widget, stretch=18)
        vert_widgets_layout.addWidget(self.temperature_widget, stretch=11)
        vert_widgets_layout.addWidget(self.depth_widget, stretch=11)
        vert_widgets_layout.addWidget(self.countdown_widget, stretch=20)

        # Setup cameras
        self.cam_down = VideoFrame()
        self.cam_back = VideoFrame()
        self.cam_front = VideoFrame()
        
        # Product demo map image
        self.demo_map = qtw.QLabel()
        # Dynamically sized, endures the image fits any aspect ratio. Will resize image to fit
        self.demo_map.setScaledContents(True)
        self.demo_map.setSizePolicy(qtw.QSizePolicy.Ignored, qtw.QSizePolicy.Ignored)

        # Initial value of CoM shift
        self.com_shift_widget.update([0.0, 0.0, 0.0])

        # (0, 0)    (0, 1)
        # (1, 0)    (1, 1)
        
        cam_layout.addWidget(self.cam_back.label, 0, 0)
        cam_layout.addWidget(self.cam_front.label, 0, 1)
        cam_layout.addWidget(self.cam_down.label, 1, 0)
        cam_layout.addWidget(self.demo_map, 1, 1)

        home_window_layout.addLayout(vert_widgets_layout, stretch=1)
        home_window_layout.addLayout(cam_layout, stretch=9)

    def tab_changed(self, index):
        """
        Triggered `currentChanged` signal which is activated when the user changes tabs.

        Sets focus to the entire tab area to avoid bug of selecting text edit areas.

        Args:
            index: Index of the currently selected tab.
        """
        for i, tab in enumerate(self.tab_dict.values()):
            if i == index:
                tab.setFocus()

        # Track states of tabs
        self.pilot_open = index == 0
        self.debug_open = index == 2

    @staticmethod
    def update_cam_img(data: Image, video_frame: VideoFrame):
        """
        Updates the display of the video on the dashboard.

        Args:
            data: ROS message of type `Image` containing the new image to display.
            video_frame: `VideoFrame` object to update.
        """
        bridge = CvBridge()
        try:
            # Create cv image from ros image then resize it to fit dashboard
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as error:
            print(f"update_cam_img() failed while trying to convert image from {data.cam_msg.encoding} to 'bgr8'.\n{error}")
            sys.exit()

        height, width, _ = cv_image.shape
        bytesPerLine = 3 * width
        # Cv image must be converted to a QImage to be displayed
        frame = qtg.QImage(cv_image.data, width, height, bytesPerLine, qtg.QImage.Format_RGB888).rgbSwapped()
        video_frame.label.setPixmap(qtg.QPixmap(frame))

    @qtc.pyqtSlot()
    def update_cam_down(self):
        """
        Slot which updates down camera image on the dashboard.
        """
        if self.pilot_open:
            TabWidget.update_cam_img(self.ros_qt_bridge.cam_down_msg, self.cam_down)
    
    @qtc.pyqtSlot()
    def update_cam_back(self):
        """
        Slot which updates back camera image on the dashboard.
        """
        if self.pilot_open:
            TabWidget.update_cam_img(self.ros_qt_bridge.cam_back_msg, self.cam_back)
    
    @qtc.pyqtSlot()
    def update_cam_front(self):
        """
        Slot which updates front image on the dashboard.
        """
        if self.pilot_open:
            TabWidget.update_cam_img(self.ros_qt_bridge.cam_front_msg, self.cam_front)

    @qtc.pyqtSlot()
    def update_pilot_tab_input_states(self):
        """
        Slot which updates gui display of input states.
        """
        input_state_dict = {
            "Bambi Mode":   self.ros_qt_bridge.input_state_msg.bambi_mode,
            "Kill Button":  self.ros_qt_bridge.input_state_msg.kill,
            "Reversed":     self.ros_qt_bridge.input_state_msg.reverse
        }
        self.state_widget.update(input_state_dict)
        self.thrt_crv_widget.update(self.ros_qt_bridge.input_state_msg.thrt_crv)

    @qtc.pyqtSlot()
    def update_pressure_msg(self):
        self.depth_widget.update(self.ros_qt_bridge.pressure_msg.depth / 1000 + 0.06)  # Convert from mm to m, display to 3 decimal places

    @qtc.pyqtSlot()
    def update_temperature(self):
        self.temperature_widget.update(self.ros_qt_bridge.temperature_msg.data)
        
    def create_debug_tab(self, tab: qtw.QWidget):
        # Setup layouts
        debug_layout = qtw.QHBoxLayout(tab)
        debug_layout.setSpacing(0)
        
        info_layout = qtw.QVBoxLayout()
        graph_layout_1 = qtw.QVBoxLayout()
        graph_layout_2 = qtw.QVBoxLayout()
        term_layout = qtw.QVBoxLayout()

        self.leak = NumericDataWidget(tab, "Leak Status", PATH + "/dash_styling/numeric_data_widget.txt", self.colors)
        self.humidity = NumericDataWidget(tab, "Humidity (%)", PATH + "/dash_styling/numeric_data_widget.txt", self.colors)
        self.barometric_pressure = NumericDataWidget(tab, "Pressure (atm)", PATH + "/dash_styling/numeric_data_widget.txt", self.colors)
        self.ambient_temperature = NumericDataWidget(tab, "Ambient Temp (°F)", PATH + "/dash_styling/numeric_data_widget.txt", self.colors)
        
        info_layout.addWidget(self.leak)
        info_layout.addWidget(self.humidity)
        info_layout.addWidget(self.barometric_pressure)
        info_layout.addWidget(self.ambient_temperature)
        
        self.cpu_usage = DynamicPlotWidget(tab, "Time", "CPU Usage", PATH + "/dash_styling/dynamic_plot_widget.txt", self.colors)
        self.memory_usage = DynamicPlotWidget(tab, "Time", "Memory Usage", PATH + "/dash_styling/dynamic_plot_widget.txt", self.colors)
        self.cpu_temperature = DynamicPlotWidget(tab, "Time", "CPU Temperature", PATH + "/dash_styling/dynamic_plot_widget.txt", self.colors)
        self.net_sent = DynamicPlotWidget(tab, "Time", "Net Megabytes Sent", PATH + "/dash_styling/dynamic_plot_widget.txt", self.colors)
        self.net_recv = DynamicPlotWidget(tab, "Time", "Net Kilobytes Received", PATH + "/dash_styling/dynamic_plot_widget.txt", self.colors)

        temp = qtw.QFrame()
        graph_layout_1.addWidget(temp, stretch=3)
        graph_layout_1.addWidget(self.cpu_usage, stretch=3)
        graph_layout_1.addWidget(self.memory_usage, stretch=3)
        graph_layout_2.addWidget(self.cpu_temperature)
        graph_layout_2.addWidget(self.net_sent)
        graph_layout_2.addWidget(self.net_recv)

        self.term_widget = TermWidget(tab, PATH + "/dash_styling/term_widget.txt", self.colors)
        term_layout.addWidget(self.term_widget)

        debug_layout.addLayout(info_layout, stretch=1)
        debug_layout.addLayout(graph_layout_1, stretch=3)
        debug_layout.addLayout(graph_layout_2, stretch=3)
        debug_layout.addLayout(term_layout, stretch=3)

    @qtc.pyqtSlot()
    def update_debug(self):
        msg = self.ros_qt_bridge.debug_msg
        # Update display if the tab is open
        if self.debug_open:
            self.cpu_usage.update(msg.time, msg.cpu_usage)
            self.memory_usage.update(msg.time, msg.memory_usage)
            self.cpu_temperature.update(msg.time, msg.cpu_temperature)
            self.net_sent.update(msg.time, msg.net_sent / 1000000)  # Convert bytes to megabyte
            self.net_recv.update(msg.time, msg.net_recv / 1000)     # Convert bytes to kilobyte
        # If the tab is not open, store the data
        else:
            self.cpu_usage.append(msg.time, msg.cpu_usage)
            self.memory_usage.append(msg.time, msg.memory_usage)
            self.cpu_temperature.append(msg.time, msg.cpu_temperature)
            self.net_sent.append(msg.time, msg.net_sent / 1000000)  # Convert bytes to megabyte
            self.net_recv.append(msg.time, msg.net_recv / 1000)     # Convert bytes to kilobyte

    @qtc.pyqtSlot()
    def update_bme280(self):
        if self.debug_open:
            msg = self.ros_qt_bridge.bme280_msg
            self.humidity.update(msg.humidity)
            self.barometric_pressure.update(msg.pressure * 0.001315)        # Convert torr to atm
            self.ambient_temperature.update(msg.temperature * 9 / 5 + 32)   # Convert C to F


class Dash(Node):
    """
    Creates and runs a ROS node which updates the PyQt dashboard with data from ROS topics.
    """

    def __init__(self, ros_qt_bridge: RosQtBridge):
        """
        Initialize 'dash' node

        Args:
            ros_qt_bridge: Bridge object for functionality between ROS and Qt.
        """
        super().__init__("dash")

        self.create_subscription(InputStates, "input_states", ros_qt_bridge.callback_input_states, 10)        
        self.create_subscription(DebugInfo, "debug_info", ros_qt_bridge.callback_debug, 10)
        self.create_subscription(Bme280, "bme280", ros_qt_bridge.callback_bme280, 10)
        self.create_subscription(Image, "camera/down/image", ros_qt_bridge.callback_cam_down, 10)
        self.create_subscription(Image, "camera/back/image", ros_qt_bridge.callback_cam_back, 10)
        self.create_subscription(Image, "camera/front/image", ros_qt_bridge.callback_cam_front, 10)
        self.create_subscription(Float32, "temperature", ros_qt_bridge.callback_temperature, 10)
        self.create_subscription(ParameterEvent, "parameter_events", ros_qt_bridge.callback_param_event, 10)
        self.create_subscription(PressureSensor, "pressure", ros_qt_bridge.callback_pressure, 10)

        ros_qt_bridge.add_publisher(self.create_publisher(String, "keystroke", 10))

        # Comment this out if we want to test the dashboard without parameters (will crash otherwise)
        ros_qt_bridge.add_set_params(SetRemoteParams(self, "pilot_input"), SetRemoteParams(self, "thrust"))

def fix_term():
    """
    If VS Code was installed with snap, the 'GTK_PATH' variable must be unset.
    This is automated in this function
    """
    if "GTK_PATH" in environ and "snap" in environ["GTK_PATH"]:
        environ.pop("GTK_PATH")


def main(args=None):
    rclpy.init(args=args)
    
    fix_term()

    app = qtw.QApplication([])

    bridge = RosQtBridge()

    pilot_dash = MainWindow(bridge)

    # Setup node
    dash_node = Dash(bridge)

    # Threading allows the process to display the dash and run the node at the same time
    # Create and start a thread for rclpy.spin function so the node spins while the dash is running
    node_thread = Thread(target=rclpy.spin, args=(dash_node,))
    node_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main(sys.argv)
