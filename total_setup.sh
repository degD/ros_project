#!/bin/bash

# ----------------------------------------------------------------------------------------------------------------------------
# PART 1: ROS NOETIC AND ROTORS_SIMULATOR
# ----------------------------------------------------------------------------------------------------------------------------

# First thing first
sudo apt install wget git

# Install and setup ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros \
    ros-noetic-mavlink python3-wstool python3-catkin-tools protobuf-compiler \
    libgoogle-glog-dev ros-noetic-control-toolbox ros-noetic-mavros
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source /opt/ros/noetic/setup.bash

# Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin init

# First build, to be sure everything is working
cd ~/catkin_ws/
catkin build

# Install rotors_simulator
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/rotors_simulator.git
git clone https://github.com/ethz-asl/mav_comm.git

# Fix a bug in the rotors_simulator hil_interface
sed -i '/typedef mavlink::mavlink_message_t mavlink_message_t;/a typedef mavlink::mavlink_status_t mavlink_status_t;' \
    ~/catkin_ws/src/rotors_simulator/rotors_hil_interface/include/rotors_hil_interface/hil_interface.h

# Final build
catkin build --jobs 4 --mem-limit 80%

# Final setup
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# ----------------------------------------------------------------------------------------------------------------------------
# PART 2: PROJECT SETUP
# ----------------------------------------------------------------------------------------------------------------------------

# Install miniconda
cd ~
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p "$HOME/miniconda3"
conda init bash
source ~/.bashrc

# Create the environment for mmrotate
conda create -n mmlab python=3.9 -y
conda activate mmlab
conda install -y pytorch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 pytorch-cuda=11.8 -c pytorch -c nvidia
pip install -U openmim
pip install fsspec
mim install -y mmcv-full==1.7.2
mim install -y mmdet==2.28.2
pip install --force-reinstall -v "numpy==1.25.2"
pip install mmrotate

# Clone and install project repository
cd ~
git clone git@github.com:degD/ros_project.git
cd ~/ros_project
mim download mmrotate --config oriented_rcnn_r50_fpn_1x_dota_le90 --dest .
echo 'export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libtiff.so.5' >> ~/.bashrc
source ~/.bashrc

# Install Python requiriments
pip install flask dotenv rospkg

# Set .env values 
MAPTILER="Write your token key here"
PLATEREC="Write your token key here"
cd ~/ros_project
touch .env
echo "$MAPTILER" >> .env
echo "$PLATEREC" >> .env

# ----------------------------------------------------------------------------------------------------------------------------
# PART 3: INITIALIZE DEMO
# ----------------------------------------------------------------------------------------------------------------------------

# Place modified files
cd ~/ros_project
bash ./place_sim_files.sh

# Start gazebo (make sure it is running)
roslaunch rotors_gazebo rotors_hovering_vi_extra_camera.launch mav_name:=firefly world_name:=test_city4

# Start web interface and draw inside the service area, press "complete", then "start"
python app.py