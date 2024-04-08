from os import path, environ

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # If VS Code was installed with snap, the 'GTK_PATH' variable must be unset.
    if "GTK_PATH" in environ and "snap" in environ["GTK_PATH"]:
        environ.pop("GTK_PATH")

    PKG_NAME = 'seahawk_description'
    pkg_path = get_package_share_path(PKG_NAME)
    model_path = path.join(pkg_path, f'urdf/{PKG_NAME}.urdf')

    return LaunchDescription([
        DeclareLaunchArgument(
                name='model',
                default_value=model_path
        ),
        Node(
            package='seahawk',
            executable='rviz_markers',
            name='rviz_markers'
        ),

       Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', path.join(get_package_share_path('seahawk'), 'rviz/debug_kinematics.rviz')]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description':  ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),

        Node(
            package='seahawk',
            executable='pilot_input',
            name='pilot_input',
            output='screen'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        Node(
            package='seahawk',
            executable='thrust',
            name='thrust',
            output='screen',
            parameters=[{
                "publishing_pwm": False
            }],
        )
    ])
