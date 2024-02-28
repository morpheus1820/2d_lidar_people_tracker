# 2d_lidar_people_tracker
![detectors_ros_image](https://github.com/morpheus1820/2d_lidar_people_tracker/assets/1096775/79d191a8-ddf1-4240-8a58-34563a4129eb)

## Install
```
mkdir -p $HOME/ros2_ws/src && cd $HOME/ros2_ws/src
git clone https://github.com/morpheus1820/2d_lidar_people_tracker
# install torch, e.g., pip3 install torch torchvision
pip install -r 2d_lidar_people_tracker/requirements.txt
cd $HOME/ros2_ws` 
colcon build && source install/setup.bash
```

## Configure
`detector_ros/config/dr_spaam_ros.yaml`
  - use_gpu: enable CUDA, default True
  - detector_model: DROW3 or DR-SPAAM, default DR-SPAAM
  - weight_file: model weights, for DR-SPAAM, default "ckpt_jrdb_ann_dr_spaam_e20.pth"

## Run
- Laser detector:

`ros2 launch detector_ros dr_spaam_ros.launch.py`

- Yolo detector:

`ros2 launch detector_ros yolo.launch.py`

- Laser + Yolo detectors:

`ros2 launch detector_ros both.launch.py`

- Tracker:

`ros2 launch tracker_ros tracker_ros.launch.py`
