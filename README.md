# AQUA SLAM

## Installation
Local Host:
```bash
mkdir aqua_slam_ws/src -p
cd aqua_slam_ws/src
git clone https://github.com/DaDa0o0/AQUA_SLAM_test.git
cd ./AQUA_SLAM_test/docker
docker compose build
docker compose up -d
xhost +
docker exec -it orb_dvl2_ros_noetic bash
```
In docker container:
```bash
catkin_make -DCMAKE_CXX_COMPILER=/usr/bin/clang++-10 DCMAKE_C_COMPILER=/usr/bin/clang-10
mkdir -p ./src/AQUA_SLAM/Vocabulary
cd ./src/AQUA_SLAM/Vocabulary
/root/.local/bin/gdown --fuzzy https://drive.google.com/file/d/1kFIh_By8wMj6AySCJ1aKjWvi7EHP7O7G/view\?usp\=sharing
cd /root/catkin_ws
source devel/setup.bash
roslaunch ORB_DVL2 blue_gx5_StructureEasy.launch
```
**Note:** We provide multiple launch files for each sequence (e.g., `blue_gx5_StructureEasy.launch` for `Structure_Easy.bag`), as some sequences use different extrinsic calibrations. Please make sure to use the corresponding launch file when running each ROS bag.

## Data Download
Please visit [Tank Dataset](https://senseroboticslab.github.io/underwater-tank-dataset/) website to download ros bags.

Here we download **Structure_Easy.bag** for test.

Place the ros bag under `aqua_slam_ws/src/AQUA_SLAM_test/dataset`

Open a new terminal on local host to play the rosbag:
```bash
docker exec -it orb_dvl2_ros_noetic bash
rosbag play ./src/AQUA_SLAM/dataset
```
