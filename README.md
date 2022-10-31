# obot

---

![obot](https://github.com/samuko-things/obot/blob/main/obot_pics1.png)

```
clone (git clone git@github.com:samuko-things/obot.git) or Download
the repo or in your ROS2 workspace, build, and source it.
```
## Basic Launch
to spawn robot in gazebo:
```shell
$ ros2 launch obot_description spawn.launch.py
```

to spawn robot in gazebo and view transforms and sensors in rviz:
```shell
$ ros2 launch obot_description spawn_and_view.launch.py
```

you can drive it around using the [pynput_teleop_twist_keyboard](https://github.com/samuko-things/pynput_teleop_twist_keyboard.git) package I wrote. test the different driving modes of the teleop package


## Slam Mapping (2D-Map Creation) Launch

![obot](https://github.com/samuko-things/obot/blob/main/obot_slam_mapping1.png)
```
install the following before you start slam mapping and 
navigation:

# install the robot localization package
	sudo apt install ros-<ros2_distro>-robot-localization

# inatall ros2 navigation packages
    sudo apt install ros-<ros2-distro>-navigation2
    sudo apt install ros-<ros2-distro>-nav2-bringup
    apt install ros-<ros-distro>-slam-toolbox
```

mapping a test world:
```shell
$ ros2 launch obot_slam_mapping slam_mapping_in_test_world.launch.py
```

mapping a simple house world:
```shell
$ ros2 launch obot_slam_mapping slam_mapping_in_simple_house.launch.py
```

you can drive it around using the [pynput_teleop_twist_keyboard](https://github.com/samuko-things/pynput_teleop_twist_keyboard.git) package I wrote. launch :
```shell
$ ros2 run pynput_teleop_twist_keyboard pynput_teleop_twist_keyboard 0.2 0.4
```
Use the [continuos and non holonomic] drive mode of the
package to drive the robot around to generate a map.


Then run this command on another terminal to save the map
```shell
$ ros2 run nav2_map_server map_saver_cli -f <path/to/save/the/mapp/[map-file-name]>
```
should see two file created in the path specified with format [map-file-name].pgm and [map-file-name].yaml

