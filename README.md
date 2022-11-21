# ROS2Plugin

Please be sure to source the ROS2 workspace in the same terminal as the one in which you are configuring, compiling and using the plugin.
If your installation is system-wide as described in [ROS2 Install Instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html):
```
source /opt/ros/$ROS2_VERSION/setup.bash
```
You can add this line to your .bashrc

In order to find the sofa lib for out of tree compilation, the environement variable `CMAKE_PREFIX_PATH` needs to be defined to point to the location of the library you want to use :
```
export SOFA_INSTALL_DIR=path_to_sofa_install_directory/lib
```
