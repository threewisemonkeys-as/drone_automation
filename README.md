# drone_automation
Controller for drone automation using MAVROS

## Contents  
- `controller.py`: Contains core controller class
- `waypoint.py`: Waypoint controller class for drone to traverse a list of points
- `depth_processor.py`: A node to process point cloud depth map
- `utils.py`: A collection of utility functions for converversion, unwrapping etc.

## Requirements 
- [ROS](http://wiki.ros.org/ROS/Installation)
- [MAVROS](https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html)
- PX4 [Firmware](https://github.com/PX4/Firmware.git)
- PX4 [Gazebo for MAVLink](https://github.com/PX4/sitl_gazebo)

You can follow the steps given [here](https://dev.px4.io/v1.8.0/en/) for setting up your environment

## Usage 

### To set up this package

```bash
cd catkin_ws/src
git clone https://github.com/threewisemonkeys-as/drone_automation.git 
cd ..
catkin build
source devel/setup.bash
```

### Demo in empty world

```bash
roslaunch drone_automation iris_empty_world.launch
rosrun drone_automation waypoint.py
```


### Demo warehouse world with lidar, camera or depth

Models for the iris quadcopter with either depth, fpv or rplidar have been included. You can demo them in the warehouse gazebo world. For example
```bash
roslaunch drone_automation iris_depth_warehouse.launch
rosrun drone_automation waypoint.py
```

The relevant topics should be displayed in RViz


## API Example

`controller.py` contains the core `Controller` class. This can be used as follows
```python
# Initialises controller object with its publishers, subscribers and clients
drone = DroneController()

# Drone takes off and lands
drone.takeoff(2)
drone.land()
```


`waypoint.py` contains a `WaypointController` class which inherits from `Controller`. It can be used to traverse a path
```python
# Initialises controller object with its publishers, subscribers and clients
wc = WaypointController()

# Drone takes off, traverses given points and lands
path = [(6, 0, 2), (6, 4, 2)]
wc.takeoff()
wc.traverse_path(path)
wc.land()
```


## TODO
- [X] Add waypoint controller
- [X] Use `setpoint_velocity` with proportional control
- [ ] Incorporate obstacle detection using LIDAR / Camera

## Resources
- [GAAS](https://gaas.gitbook.io/guide/) (Generalized Autonomy Aviation System) Guide
- PX4 Development Guide [MAVROS Example](https://dev.px4.io/v1.9.0/en/ros/mavros_offboard.html)
