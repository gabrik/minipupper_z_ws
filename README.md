# Mini Pupper ROS (noetic) + Zenoh Repository


Tested on
* Ubuntu 20.04 (ROS Noetic)

## 1.Installation Guide
**This setup corresponds to the Raspberry Pi on your Mini Pupper.**


#### 1.1 Hardware Dependencies, ROS and Zenoh

You should first install dependencies of servos, battery moniter and display screen.
See [minipupper-bsp](https://github.com/mangdangroboticsclub/minipupper-bsp).

Install ROS Noetic ROS-Base (No GUI) by following the page below.

https://wiki.ros.org/noetic/Installation/Ubuntu

Install zenoh-python APIs

```
pip3 install eclipse-zenoh
```

#### 1.2 Mini Pupper ROS+Zenoh packages installation

Install the build tools first.

```sh
sudo apt install -y python3-vcstool python3-rosdep
```


##### 1.2.1 Build packages in the Mini Pupper (RPi4)

Download the Mini Pupper ROS+Zenoh workspace

```sh
git clone https://github.com/gabrik/minipupper_z_ws
cd minipupper_z_ws/src
# it's not recommend to compile gazebo and cartographer on raspberry pi
touch champ/champ/champ_description/CATKIN_IGNORE
touch champ/champ/champ_gazebo/CATKIN_IGNORE
touch champ/champ/champ_navigation/CATKIN_IGNORE
touch minipupper_ros/mini_pupper_gazebo/CATKIN_IGNORE
touch minipupper_ros/mini_pupper_navigation/CATKIN_IGNORE
```

Build and install all ROS packages.

```sh
# install dependencies without unused heavy packages
cd ~/minipupper_z_ws
rosdep install --from-paths . --ignore-src -r -y --skip-keys=joint_state_publisher_gui --skip-keys=octomap_server
catkin_make
source ~/minipupper_z_ws/devel/setup.bash
```


##### 1.2.1 Build packages in the your PC

Download the Mini Pupper ROS+Zenoh workspace

```sh
git clone https://github.com/gabrik/minipupper_z_ws
cd minipupper_z_ws/src
```

Build and install all ROS packages.

```sh
# install dependencies without unused heavy packages
cd ~/minipupper_z_ws
rosdep install --from-paths . --ignore-src -r -y
catkin_make
source ~/minipupper_z_ws/devel/setup.bash
```

#### 1.3 Network Setup

Connect your PC and Mini Pupper to the same WiFi and find the assigned IP address with commands below.

```sh
ifconfig
```

Open the file and update the ROS IP settings with the command below.

```sh
nano ~/.bashrc
```
Then add your ROS_MASTER_URI and ip address config.

```sh
export ROS_MASTER_URI=http://${IP_ADDRESS_OF_PC}:11311
export ROS_IP=${IP_ADDRESS_OF_PC}
```

For example:

```sh
export ROS_MASTER_URI=http://192.168.1.107:11311
export ROS_IP=192.168.1.107
```

After editing, input Ctrl+X to save and exit the nano editor.

## 2.Quick Start Guide
### 2.1 Calibration

Calibrate the servo angle offset according to the following document.

https://minipupperdocs.readthedocs.io/en/latest/guide/Software.html

### 2.2 Walking

#### 2.2.1 Zenoh and Brain

**You should run these command on your PC**

```sh
zenohd
```

Then start the robot brain

```sh
roslaunch mini_pupper_bringup brain.launch
```

and the telop on another terminal

```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

#### 2.2.1 Run the base driver

**You should run this command on Mini Pupper**

```sh
roslaunch mini_pupper_bringup servos.launch
```

If you don't have LD06 LiDAR, add `lidar_connected:=false` argument like the follwoing command.

```sh
roslaunch mini_pupper_bringup servos.launch lidar_connected:=false
```
