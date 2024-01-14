# Drone-control
ROS package for drone control, used by jetson nano and pixhawk.

To program your drone, use this package. In the "declaration_all_function.hpp" file, you will find declarations for all functions along with their descriptions.

![Screenshot 2024-01-14 105015](https://github.com/Spider1097/Drone-control/assets/118929720/0767cdc4-bbde-489e-806f-227e56f2578f)

## A Set up drone workspace
First of all, we need to create a workspace.

```
mkdir -p ~/drone_ws/src
cd ~/drone_ws
catkin init
```
## Dependencies installation
Install Mavros and Mavlink.

```
cd ~/cdrone_ws
wstool init ~/drone_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```

Add to bashrc.

```
echo "source ~/drone_ws/devel/setup.bash" >> ~/.bashrc
```
Update.
```
source ~/.bashrc
```
install geographiclib dependancy.
```
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

Install Drone-control and build the package.

```
cd ~/drone_ws/src
git clone https://github.com/Spider1097/Drone-control.git
cd ~/drone_ws
catkin build
```
## Run basic code
We will use only the basic code in this tutorial; another code can be found in the 'src' folder.
Before you must install PX4.

```
cd src/Firmware/
make px4_sitl gazebo 

roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" 

rosrun drone_control test_code 
```
![image](https://github.com/Spider1097/Drone-control/assets/118929720/b275c74f-60e8-4d1f-b62e-920bb485cd39)




