# ROS Params
[ROS parameters](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html) are a way of setting attributes of a node which can be accessed and modified by the ROS network.

## Usage
### Client node
*The client node is the node which makes a request to update another node's parameters*
1. Import the `SetRemoteParams` class to any node on the deck with the following syntax. The `SetRemoteParams` class from [`set_remote_params.py`](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/blob/main/src/seahawk/seahawk_deck/set_remote_params.py) provides functionality for nodes to modify each other's [ROS parameters](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html) using [services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html). The `SetRemoteParams` class streamlines setting another node's parameters to a user-friendly abstraction to reduce redundancy, creating cleaner code and increased ease-of-use.
	```py
	from seahawk_deck.set_remote_params import SetRemoteParams
	```
2. Create an instance of the class in the `__init__()` function, and pass it `self` (the current node) and the name of the node of whose parameters shall be set.
	```py
	self.set_params = SetRemoteParams(self, "other_node_name")
	```
3. In a callback, choose the parameters to set and their new values by calling `update_params()` with arguments of the name of the parameter to set and its new value. This may be called multiple times to set multiple parameters.
	```py
	self.set_params.update_params("param_name", param_val)
	```
4. Once all the parameter values have been instantiated using `update_params()`, send the parameters via service to the other node by calling `send_params()`
	```py
	self.set_params.send_params()
	```

### Service node
*The service node is the node whose parameters are being set*
1. Import required libraries for handling ROS parameters.
    ```py
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import SetParametersResult
    ```
2. Within the `__init__()` function declare a parameter. Below is an example from [`pilot_input.py`](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/blob/main/src/seahawk/seahawk_deck/pilot_input.py), additional examples can be found [here](https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/#Setup_code_and_declare_ROS2_params_with_rclpy).
    ```py
    # Parameter name is throttle_curve_choice
    # Its initial value is 1
    self.declare_parameter("throttle_curve_choice", 1)
    ```
3. Create a parameter callback using `add_on_set_parameters_callback`.
    ```py
    # self.update_key_stroke is the function called,
    # it can have any name deemed descriptive
    self.add_on_set_parameters_callback(self.update_key_stroke)
    ```
4. Write the function called when a parameter is updated. Add any logic deemed necessary. It is recommended to store the parameter in an attribute which is then accessed by the class if the value is used outside the callback. Below is an example from [`pilot_input.py`](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/blob/main/src/seahawk/seahawk_deck/pilot_input.py).
    ```py
    def update_key_stroke (self, params: list[Parameter]) -> SetParametersResult:
        """
        Callback for parameter update. Updates the Center of Mass offset 
        and the motor and inverse config afterwards.

        Args:
            params: List of updated parameters (handles by ROS2)

        Returns:
            SetParametersResult() which lets ROS2 know if the parameters were 
            set correctly or not
        """
        for param in params:
            if param.name == "throttle_curve_choice":
                if (value:=param.value) in {1, 2, 3}:
                    # Note: If this is causing issues, maybe move to a separate callback
                    # See: https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html#parameter-callbacks
                    self.key_input = value
                    return SetParametersResult(successful=True)
        return SetParametersResult(successful=False))
    ```

## Other notes for ROS parameters
### param list
This command will list all of the available parameters on a given node, or on all discoverable nodes if no node is given.
```sh
ros2 param list node
ros2 param list
``` 
**Example:**
```sh
❯ ros2 param list thrust
  center_of_mass_offset
  use_sim_time
```

### param describe
This command describes a specific parameter from a node.
```sh 
ros2 param describe node param
```
**Example:** 
```sh 
❯ ros2 param describe thrust center_of_mass_offset
Parameter name: center_of_mass_offset
  Type: double array
  Constraints:
```

### param dump
This command outputs a list of a node's parameters and those parameter's attributes in a YAML file format. 
```sh
ros2 param dump node
```
**Example:**

```sh
❯ ros2 param dump thrust
/thrust:
  ros__parameters:
    center_of_mass_offset:
    - 0.0
    - 0.0
    - 0.0
    use_sim_time: false
```
