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
you can drive it around using the teleop_twist_keyboard package or you can 
also use the [pynput_teleop_twist_keyboard](https://github.com/samuko-things/pynput_teleop_twist_keyboard.git) package I wrote.
