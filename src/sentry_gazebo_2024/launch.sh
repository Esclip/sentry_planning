cd /media/dragon/EXTERNAL_USB/规划仿真环境
ls -a
source /media/dragon/EXTERNAL_USB/规划仿真环境/devel/setup.bash
# roslaunch mbot_gazebo mbot_3d_lidar_gazebo.launch
# roslaunch mbot_control mbot_control.launch
# roslaunch mbot_teleop mbot_teleop.launch
gnome-terminal -t "gazebo.bash"  -x  bash -c "source devel/setup.bash&&roslaunch mbot_gazebo mbot_3d_lidar_gazebo.launch;exrc bash"
sleep 5s
gnome-terminal -t "control.bash"  -x bash -c "source devel/setup.bash&&roslaunch mbot_control mbot_control.launch;exrc bash"
sleep 5s
gnome-terminal -t "teleop.bash"  -x bash -c "source devel/setup.bash&&roslaunch mbot_teleop mbot_teleop.launch;exrc bash"