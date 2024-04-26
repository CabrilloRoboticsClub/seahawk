*from 2023*
# Miscellaneous Notes on the ROS Packaging Process

## Tools and Files

### package.xml

This file lives at the top of a ROS package, and defines the metadata for it. It's job is to contain package name, version, maintainers, upstream URL's, etc, as well as the package dependencies. These dependencies are specified as names understood by the [rosdep](#rosdep)tool.

The file seems to be technically optional, as the various build tools have some capacity to infer the package information from other sources (CMakeLists.txt, or setup.py in the case of `ament_python`).

### ament

Ament is a ROS 1 & 2 build tool. Historically, ROS has used CMake-based build processes, and `ament` can still handle this. Additionally, it is capable of Python-only build systems. This appears to happen through Python's `setuptools` module and it's `setup.py` file.

>It's not clear to me at this time if `ament` reads the contents of `setup.py` directly, or if it simply invokes `setuptools`. I believe the package-inference abilities are done by reading `setup.py`, but the actual build process is handled through `setuptools`. If this is the case, we can switch to a `pyproject.toml` description and still have successful builds -- although we'd lose the ability to infer what whould otherwise be explicitly stated in the [package.xml](#packagexml) file (see above). This is an acceptable trade. We're already supplying the package.xml file, so we don't need to have our build tool infer anything about it.

Package initialization can be done by calling

`:~$ ros2 pkg create --build-type <type> <package_name>`

Where build-type is "ament_cmake" or "ament_python", and the package name is, of course, the name of the new package.

### colcon

Colcon is the shiny new build tool to handle all package formats and systems that ROS has collected over the years. Tools like `ament` and `catkin` are meant to only operate on a single package at a time, so building a whole workspace full of pieces is fiddly and annoying.

There exists a [design doc](https://design.ros2.org/articles/build_tool.html) on the ROS2 org site that explains it quite well. It also provides background for the other tools.

> It is not clear to me if `colcon` replaces `ament` and `catkin` in their entireties, or if it simply puppets them to orchestrate a workspace-wide build. So far, we seem to have `ament` and it's components installed, though I'm not sure if we did that, or if the `colcon` package is marked as depending on `ament`.

### rosdep

Rosdep resolves packages. Since ROS runs on multiple platforms, the names of the system packages to supply various components are not likely to be identical. The rosdep tool gets around this by keeping a list of key:values sets. You can specify the ROS package name (the rosdep key) and then ask rosdep to collect whatever `apt` or `yum` package supplies that for your system. A package's[package.xml](#packagexml) file contains a list of dependencies for building, running, and testing.

For example: A package we're building depends on ROS2's `example_messages` package. This information is recorded in that package's `package.xml` file between some `<build_depend>` XML tags. They can be installed into the system with a call to `rosdep install -i --from-path src/$that-package --rosdistro humble -y` from the root of the repository.

> To install *all* dependencies of the repo (at least according to the package.xml), replace the `... --from-path src/$that-package ...` with just `... --from-path src/ ...`. Drilling down into sub-packages works the same way: Specify more-, or less-specific paths to only get those things.

#### rosdep keys

> The official docs have more info. See ["how do I use the rosdep tool"](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Rosdep.html#how-do-i-use-the-rosdep-tool).

1. Run `# rosdep init` to populate `/etc/ros/rosdep/sources.list.d/20-default.list`
2. Run `$ rosdep update` to read those sources and update the pacakge keys -- these get stored to `$HOME/.ros/rosdep/sources.cache/`
3. Inspect contents of `$HOME/.ros/rosdep/sources/cache/index` for a list of yaml files.
4. Find your package name in a yaml file -- now you know what name ROS has for that package (put it in your package.xml depends lists!)

It seems that `ament_python` is absent from the core ros humble list, while `ament_cmake`, and it's components/extensions are present.

What's super cool, though, is that there's a `rosdep/python.yaml` file! We *can* tell the package.xml file about our pip dependencies and let `rosdep` collect them for us.
