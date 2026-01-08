# AQUA SLAM
## Installation
```bash
mkdir aqua_slam_ws/src -p
cd aqua_slam_ws/src
git clone https://github.com/DaDa0o0/AQUA_SLAM_test.git
cd ./AQUA_SLAM_test/docker
docker compose build
docker compose up -d
docker exec -it orb_dvl2_ros_noetic bash
catkin_make -DCMAKE_CXX_COMPILER=/usr/bin/clang++-10 DCMAKE_C_COMPILER=/usr/bin/clang-10
source devel/setup.bash
roslaunch ORB_DVL2 blue_gx5.launch
```

