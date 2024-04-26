"""
input_xbox_one.py

Handle input from an Xbox One controller

Copyright (C) 2022-2023 Cabrillo Robotics Club

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Cabrillo Robotics Club
6500 Soquel Drive Aptos, CA 95003
cabrillorobotics@gmail.com
"""

# For reading argv
import sys
# for reading in command line arguments in python

# Ros client libary imports
import rclpy
from rclpy.node import Node 
# essential improts for creating ROS node functionality

# ROS messages imports
from geometry_msgs.msg import Twist 
# imports messages of type Twist from the geometry_msgs module
from sensor_msgs.msg import Joy
# imports joystick messages of type Joy from sensor_msgs module
from std_msgs.msg import Bool
# imports Bool type messages from std_msgs module

# ****** Q & A ******
#
#
# Q: why do we put 'def' before functions? is it like typename <T> from C++?
# A: 'def' is simply a way to define a function in python. dont need to include return type unless
# its for making the code more easily readible as python handles types for you. i dont like...
#
#
# Q: why do so many functions and variables start with '__'?
# A: 

# Q: What is a module?

class StickyButton():  # begin creation of StickyButton class. Not a super or sub class.
    """
    Class that implements sticky buttons, meaning a button is pressed to turn it on, 
    and pressed again to turn it off
    """

    def __init__(self):  # default constructor for StickyButton class
        # Q: what does it mean to put (self) in function parameter?

        """
        Initialize 'StickyButton' object
        """

        self.__feature_state = False
        # creates a boolean variable called __feature_state that determines if button is pressed (True) or not (False)
        # 'self.' implies that this is an instance variable of the current instance of this class

        # button bouncing is when the button bounces rly quick after being pressed, showing incorrect data.

        self.__track_state = 0b0000
        # creates an integer variable called __track_state that reps a binary number
        # likely signifies the state of the button using binary bit information
        # Q: How does it do this specifically??
        # A: the binary number tracks last 4 states of button
        # sort of like an array 0b0000
    
    def check_state(self, cur_button_state: bool) -> bool:
        # check_state function that checks is a button is on or off on the controller
        # 'cur_button_state: bool' is a type hint that indicates cur_button_state is of type 'bool'
        # '-> bool' indicates the return type of the function to make reading the code easier

        """
        Checks if a button is toggled on or off and accounts for debouncing

        Args:
            cur_button_state: Current state of the button provided by the controller

        Returns:
            True if the button is toggled on, False if off
        """

        # Q: where is the cur_button_state data coming from?

        # Append the current button state to the tracker then remove the leftmost bit
        # such that self.__track_state records the most recent four states of the button
        self.__track_state = (self.__track_state << 1 | cur_button_state) & 0b1111
        # Q: someones gonna need to explain this to me...
        # A: this is bitshifting. its moving from the binary representation of one number to another using an addition of one bit.
        # "|" this is bitwise OR which OR's two bitwise numbers like a boolean
        # "&" is a bitwise AND with 0b1111, it gets rid of...

        # Account for bounce by making sure the last four recorded states
        # appear to represent a button press. If so, update the feature state
        # Q: what is bounce?
        if self.__track_state == 0b0011:  # if we have two off states '00' followed by two on states '11' button is probs not bouncing 
            self.__feature_state = not self.__feature_state  # reverses polarity?
        return self.__feature_state

    def reset(self):
        """
        Resets button state to original configuration
        """
        self.__feature_state = False
        self.__track_state = 0b0000

# Q: So are we creating a node inside of a node here?
# A: nope lol just inherticance

class InputXboxOne(Node):  # creates a subclass InputXboxOne of superclass Node from ROS2 library. basically inheritance
    """
    Class that implements the joystick input
    """

    def __init__(self):  # default construcutor for current instance of InputXboxOne
        # Q: why passing 'self' as parameter?
        # A: unless passing in self, the method is static. by passing in self you are giving fuction access to members of specific instance of class.

        """
        Initialize 'input_xbox_one' node
        """

        super().__init__("input_xbox_one")
        # initializes the input_xbox_one node by sending name of node to super class Node
        # Q: Why do we do this? Isnt class InputXboxOne(Node); enough?
        # A: to initialize node, there is logic that must be defined. by calling super().__init__ we're doing all set up to start this node. helpful black box.

        self.create_subscription(Joy, "joy", self.__callback, 10)
        # creates sub for current instance of class with self.create_subscription
        # 'Joy' represents the message type and "joy" is topic from which joystick messages are published
        # self.__callback is causing this to refresh whenever theres a new joystick message
        # 10 is a QoS setting for the sub. mystery black box....
        # PURPOSE: this is designed to recieve joystick data from xbox_one controller

        self.__twist_pub = self.create_publisher(Twist, "desired_twist", 10)
        # sets up a publisher message for the current instance of the class
        # publishes Twist type messages to the desired_twist topic 
        # 10 is the QoS setting for the pub
        # doing self.variable makes the variable a memeber variable of the class. '__' in front of var makes it priv
        # PURPOSE: Not sure...

        self.__claw_pub = self.create_publisher(Bool, "claw_state", 10)
        # sets up a publisher message for the current instance of the class
        # publishes Bool type messages to the claw_state topic 
        # 10 is the QoS setting for the pub
        # PURPOSE: this published information about whether or not the claw is open or closed
        
        self.__buttons = {
            # "" :              StickyButton(),     # left_stick_press
            # "" :              StickyButton(),     # right_stick_press
            "claw":             StickyButton(),     # a
            "bambi_mode":       StickyButton()      # b
            # "":               StickyButton(),     # x
            # "":               StickyButton(),     # y
            # "":               StickyButton(),     # window
            # "":               StickyButton(),     # menu
        }
        # a dictionary of key value pairs for mapping out relevant buttons on the controller
        # each key is a string representing a button and its value is an instance of StickyButton class
        # PURPOSE: keep track of whether or not a button is pressed or not
        # Q: how does the StickyButton class work?

    def __callback(self, joy_msg: Joy):
        # declares a function that takes its current instance and a message of type Joy from the joy_msg topic
        # this function basically gathers joystick input data
        # the 'joy_msg: Joy' basically means the variable is named 'joy_msg' with type 'Joy'

        """
        Takes in input from the joy message from the x box and republishes it as a twist specifying 
        the direction (linear and angular x, y, z) and percent of max speed the pilot wants the robot to move

        Args:
            joy_msg: Message of type 'Joy' from the joy topic
        """

        # Debug output of joy topic
        # Q: what does 'debug' mean in this case?
        # A: basically debug is a conditional cout statement

        # Q: where does the function get_logger() & debug() come from? Is it a function part of the 'Node' super class?
        # A: handles ROS specific print message

        self.get_logger().debug(f"Joystick axes: {joy_msg.axes} buttons: {joy_msg.buttons}")
        # 'f"Joystick axes: {joy_msg.axes} buttons: {joy_msg.buttons}"' is a f-string for string formatting
        # Q: where does the '.axes' and '.buttons' come from? are they variables that are part of something
        # that has to do with the joy_msg topic?
        # Q: what are '.axes' and '.buttons'?
        # A: basically takes info from ROS class thats a black box that takes input from xbox controller both in terms of 
        # joystick message & buttons and we can retrieve those messages using .axes & .buttons

        # Map the values sent from the joy message to useful names
        controller = {
            # Left stick
            "linear_y":         joy_msg.axes[0],                # left_stick_x
            "linear_x":         joy_msg.axes[1],                # left_stick_y
            # "":               joy_msg.buttons[9],             # left_stick_press
            # Right stick
            "angular_z":        joy_msg.axes[3],                # right_stick_x
            "angular_y":        joy_msg.axes[4],                # right_stick_y
            # "":               joy_msg.buttons[10],            # right_stick_press
            # Triggers
            "neg_linear_z":     joy_msg.axes[2],                # left_trigger
            "pos_linear_z":     joy_msg.axes[5],                # right_trigger
            # Dpad
            # "":               int(max(joy_msg.axes[7], 0)),   # dpad_up
            # "":               int(-min(joy_msg.axes[7], 0)),  # dpad_down
            # "":               int(max(joy_msg.axes[6], 0)),   # dpad_left     
            # "":               int(-min(joy_msg.axes[6], 0)),  # dpad_right
            # Buttons
            "claw":             joy_msg.buttons[0], # a
            "bambi_mode":       joy_msg.buttons[1], # b
            # "":               joy_msg.buttons[2], # x
            # "":               joy_msg.buttons[3], # y
            "pos_angular_x":    joy_msg.buttons[4], # left_bumper
            "neg_angular_x":    joy_msg.buttons[5], # right_bumper
            # "":               joy_msg.buttons[6], # window
            # "":               joy_msg.buttons[7], # menu
            "reset":            joy_msg.buttons[8], # xbox
        }
        # this is another dictionary with key value pairs.
        # in this case, the key is the button on the controller, and the value is
        # a value in an array that holds data about something axes and buttons...?


        # Bambi mode cuts all twist values in half for more precise movements
        bambi_div = 2 if self.__buttons["bambi_mode"].check_state(controller["bambi_mode"]) else 1

        # Create twist message
        twist_msg = Twist()
        twist_msg.linear.x  = controller["linear_x"]     / bambi_div     # Z (forwards)
        twist_msg.linear.y  = -controller["linear_y"]    / bambi_div     # Y (sideways)
        twist_msg.linear.z  = ((controller["neg_linear_z"] - controller["pos_linear_z"]) / 2) / bambi_div # Z (depth)

        # Roll is activated at a constant 0.5 throttle if either the left or right button is pressed
        roll_dir = -1 if controller["neg_angular_x"] else 1
        twist_msg.angular.x = (roll_dir * 0.5 / bambi_div) if controller["pos_angular_x"] or controller["neg_angular_x"] else 0.0 # R (roll)
    
        twist_msg.angular.y = controller["angular_y"]    / bambi_div     # P (pitch) 
        twist_msg.angular.z = -controller["angular_z"]   / bambi_div     # Y (yaw)
     
        # Publish twist message
        self.__twist_pub.publish(twist_msg)

        # Create claw message
        claw_msg = Bool()
        claw_msg.data = self.__buttons["claw"].check_state(controller["claw"])

        # Publish claw message
        self.__claw_pub.publish(claw_msg)

        # If the x-box button is pressed, all settings get reset to default configurations
        if controller["reset"]:
            self.__buttons["bambi_mode"].reset()

    # make sure the throttle curves have rotational symmetry about the origin. -1 & 1 should remain those numbers and 0 to remain as zero.


def main(args=None):  # args = None means there are no command line arguments
    rclpy.init(args=args)  # ROS initializer for nodes
    rclpy.spin(InputXboxOne())  # keep InputXboxOne node running for as long as needed like an infinite loop. killed with cntrl c
    rclpy.shutdown()  # shut down the node when not needed


if __name__ == "__main__":
    main(sys.argv)  # extracts command line arguments & calls main function jumpts to l.250
# Q: what is this? ^
