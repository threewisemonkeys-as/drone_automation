# drone_automation
Controller for drone automation using MAVROS

## Files  
- `controller.py`: Contains core controller class
- `waypoint.py`: Waypoint controller class for drone to traverse a list of points
- `utils.py`: A collection of utility functions for converversion, unwrapping etc,

## Requirements 
- [ROS](http://wiki.ros.org/ROS/Installation)
- [MAVROS](https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html)
- PX4 [Firmware](https://github.com/PX4/Firmware.git)
- PX4 [Gazebo for MAVLink](https://github.com/PX4/sitl_gazebo)

## Usage 

### Default PX4 Empty World

To test in the default px4 empty world first run:
```bash
$ roslaunch px4 mavros_posix_sitl.launch
```

Then in a seperate terminal:
```bash
$ python waypoint.py
```

### Warehouse world with lidar or camera 

To use the either the `iris_fpv_cam` model or `iris_rplidar` model, follow this procedure - 

***Note:*** You will need to replace `<path_to_Firmware>` with the approproate path in the commands below.

1. Copy the required launch files into the px4 package:
```back
$ cp launch/* <path_to_Firmware>/launch/
```

2. Copy the required world files into px4 package:
```bash
$ cp world/* <path_to_Firmware>/Tools/sitl_gazebo/worlds/
```

3. Copy the required rviz files into px4 package:
```bash
$ mkdir -p <path_to_Firmware>/Tools/sitl_gazebo/rviz/
$ cp rviz/* <path_to_Firmware>/Tools/sitl_gazebo/rviz/
```

4. Launch the desired configuration:
```bash
$ roslaunch px4 test_iris_fpv_cam.launch
```
or
```bash
$ roslaunch px4 test_iris_rplidar.launch
```

5. Run the waypoint controller:
```bash
$ python waypoint.py
```

The camera / lidar topics should be displayed in RViz


## TODO
- [X] Add waypoint controller
- [X] Use `setpoint_velocity` with proportional control
- [ ] Incorporate obstacle detection using LIDAR / Camera

## Resources
- [GAAS](https://gaas.gitbook.io/guide/) (Generalized Autonomy Aviation System) Guide
- PX4 Development Guide [MAVROS Example](https://dev.px4.io/v1.9.0/en/ros/mavros_offboard.html)
