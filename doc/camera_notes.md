*from 2023*
# Camera Notes

## Camera Nodes

There are two camera nodes that we can use. ROS2 is distributed with a stable [v4l2 camera node](https://index.ros.org/r/v4l2_camera/). To use the packaged node install the following:

```console
sudo apt install ros-humble-v4l2-camera ros-humble-image-transport-plugins v4l2-utils
```

To run `v4l2_camera_node`:

```console
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:='/dev/video0' -p image_size:='[640,480]' 
```

The `v4l2_camera_node` doesn't seem to have the built-in ability to change the frame rate. You can do that with `v4l-ctl`:

```console
v4l2-ctl -d /dev/video0 -p 10
```

If the `ros-humble-image-transport-plugins` package is installed compressed transport will be automatically available.

The [h264_image_transport](https://github.com/clydemcqueen/h264_image_transport) package contains a node that accesses a v4l2 camera and sends h264 frames over the wire and a node that decodes the h264 frames into images. Run them like this:

```console
rov$ ros2 run image_transport h264_cam_node --ros-args -p input_fn:='/dev/video0' -p fps:=30 -p size:='640x480'

deck$ ros2 run image_transport republish h264 raw --ros-args -r in/h264:=/image_raw/h264 -r out:=/repub_raw
```

In order to compile the `h264_image_transport` node the following packages have to be installed:

```console
sudo apt install libavdevice-dev libavformat-dev libavcodec-dev libavutil-dev libswscale-dev
```

## Pi Camera Module 2.1 on RPi4

The following configuration must be present on the Pi in order to use the `imx219` camera that's in the [Pi Camera Module 2](https://www.raspberrypi.com/products/camera-module-v2/) with Video4Linux on Ubuntu:

```conf
camera_auto_detect=0
display_auto_detect=1
start_x=1
```
