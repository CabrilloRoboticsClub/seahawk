import re
import pathlib
import subprocess

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch.event_handlers import OnShutdown

# Camera paths
back_camera_path    = '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index2'
front_camera_path     = '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index2'
down_camera_path   = '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index2'

microros_serial_device = "/dev/ttyS0"
subprocess.run('sudo /usr/local/bin/openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program pico/seahawk.elf verify reset exit"',
                shell=True,
                check=True)

def generate_launch_description():
    respawn_time = 0
    nodes = [
        Node(
            package='seahawk',
            executable='debug',
            name='debug',
            output='screen',
            respawn=True,
            respawn_delay=respawn_time
        ),
        Node(
            package='seahawk',
            executable='claws',
            name='claws',
            output='screen',
            respawn=True,
            respawn_delay=respawn_time
        ),
        Node(
            package='seahawk',
            executable='i2c',
            name='i2c',
            output='screen',
            respawn=True,
            respawn_delay=respawn_time
        ),
        Node(
            package='seahawk',
            executable='servo',
            name='servo',
            output='screen',
            respawn=True,
            respawn_delay=respawn_time
        ),
    ]

    if down_camera_path is not None and pathlib.Path(down_camera_path).exists():
        nodes.append(
            Node(
                package='h264_image_transport',
                executable='h264_cam_node',
                name='down_camera',
                output='both',
                respawn=True,
                respawn_delay=respawn_time,
                parameters=[{
                    'input_fn': str(pathlib.Path(down_camera_path).resolve()),
                    'fps': 30,
                    'size': '640x480',
                    'frame_id': 'down_camera',
                    'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
                    'qos_overrides./parameter_events.publisher.history': 'keep_last',
                    'qos_overrides./parameters_events.publisher.durability': 'volatile',
                    'qos_overrides./parameter_events.publisher.depth': 1,
                }],
                remappings=[
                    ('image_raw/h264', 'camera/down/h264'),
                ],
            ),
        )

    if pathlib.Path(back_camera_path).exists():
        nodes.append(
            Node(
                package='h264_image_transport',
                executable='h264_cam_node',
                name='back_camera',
                output='both',
                respawn=True,
                respawn_delay=respawn_time,
                parameters=[{
                    'input_fn': str(pathlib.Path(back_camera_path).resolve()),
                    'fps': 30,
                    'size': '640x480',
                    'frame_id': 'back_camera',
                    'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
                    'qos_overrides./parameter_events.publisher.history': 'keep_last',
                    'qos_overrides./parameters_events.publisher.durability': 'volatile',
                    'qos_overrides./parameter_events.publisher.depth': 1,
                }],
                remappings=[
                    ('image_raw/h264', 'camera/back/h264'),
                ],
            ))
    else:
        print("back camera not found!")

    if pathlib.Path(front_camera_path).exists():
        nodes.append(
            Node(
                package='h264_image_transport',
                executable='h264_cam_node',
                name='front_camera',
                output='both',
                respawn=True,
                respawn_delay=respawn_time,
                parameters=[{
                    'input_fn': str(pathlib.Path(front_camera_path).resolve()),
                    'fps': 30,
                    'size': '640x480',
                    'frame_id': 'front_camera',
                    'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
                    'qos_overrides./parameter_events.publisher.history': 'keep_last',
                    'qos_overrides./parameters_events.publisher.durability': 'volatile',
                    'qos_overrides./parameter_events.publisher.depth': 1,
                }],
                remappings=[
                    ('image_raw/h264', 'camera/front/h264'),
                ]
            ))
    else:
        print("front camera not found!")

    if microros_serial_device is not None and pathlib.Path(microros_serial_device).exists():
        nodes.append(
            ExecuteProcess(
                cmd=[[
                    FindExecutable(name='docker'),
                    " run " ,
                    " --rm ",
                    " --name micro-ros-agent ",
                    f" --device {microros_serial_device} ",
                    " --network host ",
                    " microros/micro-ros-agent:humble ",
                    f" serial --dev {microros_serial_device} baudrate=115200 ",
                ]],
                shell=True,
                name="micro-ros-agent",
                output='both',
                respawn=True,
                respawn_delay=respawn_time
            ),
        )

        nodes.append(
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=lambda event, ctx: subprocess.run("docker kill micro-ros-agent", shell=True)
                )
            ),
        )
    else:
        print("No micro-ros serial device.")

    return LaunchDescription(nodes)
