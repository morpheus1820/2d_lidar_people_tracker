# 2d_lidar_people_tracker

## Install
```
mkdir -p $HOME/ros2_ws/src && cd $HOME/ros2_ws/src
git clone https://github.com/morpheus1820/2d_lidar_people_tracker
pip install -r 2d_lidar_people_tracker/requirements.txt
cd $HOME/ros2_ws` 
colcon build && source install/setup.bash
```

## Run
- Laser detector:

`ros2 launch detector_ros dr_spaam_ros.launch.py`

- Yolo detector:

`ros2 launch detector_ros yolo.launch.py`

- Laser + Yolo detectors:

`ros2 launch detector_ros both.launch.py`

- Tracker:

`ros2 launch tracker_ros tracker_ros.launch.py`
