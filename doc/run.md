# Running the project
## 1 Make
The ROS nodes are built with the `colcon` program. A `makefile` is present so we do not have to remember those commands. This should be ran once per session developing in a separate terminal in the `cabrillo_rov_2023` directory. You will only need to rerun `make` during that session if you modify the launch/setup files.
```sh
make
```
The build artifacts are in:

- `build`
- `install`
- `log`

Clean the build with:

```sh
make clean 
```

## 2 Source
1. Open a new terminal.
1. Source ROS. It is recommended to put this in your `.shell_name.rc` file so you do not need to run this every time you open a new terminal.
    ```sh
    source /opt/ros/humble/setup.shell_name
    # Where shell_name is the name of your shell (zsh, bash, etc)
    ```
3. Source the local setup created by `make`. This should be ran for each new terminal in the `cabrillo_rov_2023` directory.
    ```sh
    source ./install/local_setup.shell_name
    # Where shell_name is the name of your shell (zsh, bash, etc)
    ```


## 3 Launch Files 
We've migrated all the launch files to their respective ROS packages. Launch only after completing the steps above in the the `cabrillo_rov_2023` directory.

```sh 
ros2 launch seahawk rov.launch.py  
```

```sh 
ros2 launch seahawk deck.launch.py  
```

## 3 Running Individual Nodes
Nodes can be ran individually using the following syntax, where `pkg_name` is the name of the package (this will usually be `seahawk`) and `node_name` is the name of the node to run. Run nodes only only after completing the steps above in the the `cabrillo_rov_2023` directory.
```sh
ros2 run pkg_name node_name
```

**Example:**
```sh
ros2 run seahawk pilot_input
```

## 3 Run RViz
See [rviz.md](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/blob/main/doc/rviz.md)