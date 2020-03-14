
INSTALL Instruction:
sudo apt-get remove ros-*
sudo apt-get update
sudo apt-get upgrade
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh
chmod 755 ./install_ros_kinetic.sh
bash ./install_ros_kinetic.sh

sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers


cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/catkin_ws
catkin_make
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git
cd ..
catkin_make


sudo apt-get install ros-kinetic-image-transport ros-kinetic-cv-bridge ros-kinetic-vision-opencv python-opencv libopencv-dev ros-kinetic-image-proc


sudo apt-get install -y libgazebo7-dev
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro kinetic
cd ~/catkin_ws/
catkin_make
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-robot-state-publisher
sudo apt install ros-kinetic-rviz
sudo apt-get update

Run Instruction:

roslaunch final_project track.launch
roslaunch final_project moveDist.launch 
roslaunch final_project moveServer.launch
