# Joystick Recording 

This is a recording of a joystick. It makes the following moves:

1. Left Stick 
    1. Up and back
    1. Down and back
    1. Left and back 
    1. Right and back 
    1. Slow circle 
1. Right Stick 
    1. Up and back
    1. Down and back
    1. Left and back 
    1. Right and back 
    1. Slow circle 
1. Left trigger pull and release
1. Right trigger pull and release
1. Left trigger pull and release
1. Right trigger pull and release

The total play time is about 52 seconds. 

## Using this Bag 

The bag replaces the need for a real joystick. Test the `thrust` node by running the `input_xbox_one` node without the `joy` node. This bag works in place of the `joy` node. 

Start the input node: 

```console
$ ros2 run seahawk input_xbox_one
```

Start the thrust node: 

```console
$ ros2 run seahawk thrust
```

Play the bagfile: 

```console
$ ros2 bag play bags/joy_recording/
```

