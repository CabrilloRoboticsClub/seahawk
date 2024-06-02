# ROS Bags
ROS bags used to record and the replay messages sent over a topic. This can be useful for debugging behavior after it happens or for remote development. 

## Project ROS bags 
| Content | Bag | Documentation | 
| ----- | ------ | ----- |
| H264 recording | [h264_recording](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/tree/docs/bags/h264_recording) | [h264_recording.md](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/blob/main/bags/h264_recording/h264_recording.md)
| IMU recording | [imu_recording](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/tree/main/bags/imu_recording) | None
| Joystick recording | [joy_recording](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/tree/docs/bags/joy_recording) | [joy_recording.md](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/blob/main/bags/joy_recording/joy_recording.md)

## Creating a ROS bag
To create a ROS bag use `ros2 bag record`. Specify the name of the topic (`/topic`) to record and the destination of the recording (`bag_name`) as shown in the following command.
```
ros2 bag record -o bag_name /topic
```
**Example:**
```sh
ros2 bag record -o h264_recording /image_raw/h264
```
Kill the process with `CTRL-C` when completed.

## Running a ROS bag
A ROS bag is played using `ros2 bag play`.
```sh
ros2 bag play bags/h264_recording
```
