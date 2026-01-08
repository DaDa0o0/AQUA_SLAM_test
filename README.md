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
cd ./src/AQUA_SLAM/Vocabulary/
/root/.local/bin/gdown --fuzzy https://drive.google.com/file/d/1kFIh_By8wMj6AySCJ1aKjWvi7EHP7O7G/view\?usp\=sharing
cd /root/catkin_ws
source devel/setup.bash
roslaunch ORB_DVL2 blue_gx5.launch
```