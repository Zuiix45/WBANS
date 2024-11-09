<h1>
Waypoint Based Autonomous Navigation System
</h1>

- This repository contains a ros2 workspace for navigating a drone with pre-defined waypoints.
- Utilizing XRCE-DDS to connect ros2 and px4.

To build the code:
```
colcon build
```

To run the code:
```
source install/local_setup.bash
ros2 launch drone drone.launch.py
```