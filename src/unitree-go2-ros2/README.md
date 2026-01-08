# unitree go2 ros2 - champ config

> This package contains the configuration files for the Unitree Go2 robot configured with the CHAMP controller in ROS 2 (humble). It includes development of config package along with upgrade in robot description model for ROS 2 Humble distribution.

## Unitree Go2:
<div style="display: flex; gap: 50px;">
  <img src="https://oss-global-cdn.unitree.com/static/c487f93e06954100a44fac4442b94d94_288x238.png" width="250" />
  <img src=".docs/gazebo_launch.png" width="350" /> 
</div>

> Unitree Robotics is focusing on the R&D, production, and sales of consumer and industry-class high-performance general-purpose legged and humanoid robots, six-axis manipulators, and so on. We attaches great importance to independent research and development and technological innovation, fully self-researching key core robot components such as motors, reducers, controllers, LIDAR and high-performance perception and motion control algorithms, integrating the entire robotics industry chain.

## CHAMP Controller:
![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/robots.gif)

> CHAMP is an open source development framework for building new quadrupedal robots and developing new control algorithms. The control framework is based on [*"Hierarchical controller for highly dynamic locomotion utilizing pattern modulation and impedance control : implementation on the MIT Cheetah robot"*](https://dspace.mit.edu/handle/1721.1/85490).

## Resources:
- [go2 description (URDF model)](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2_description) 
- [champ robot (configs)](https://github.com/chvmp/robots)
- [champ controller](https://github.com/chvmp/champ)

## Tested on:
- Ubuntu 22.04 (ROS2 Humble)

## Current state of package:

- &check; Configure go2 robot with champ config
- &check; Robots Configurations.
    - &check; Porting of robot description packages to ROS 2.
    - &check; Porting of robot URDF to ROS2 (add new ros2_control tag).
    - &check; Porting of robot configurationf to ROS2.
    - &check; Porting of robot launch Files to ROS2.
- &check; Upgrade go2 description model for ros2 humble
- &check; Spawning go2 in gazebo environment.
- &check; Working rviz only demo.
- &check; Working Gazebo with teleoperated robot.
- &check; Adding IMU and 2D LiDAR.
- &check; Adding 3D LiDAR (Velodyne).
- &cross; Working Gazebo demo with SLAM.
- &cross; Working Gazebo demo with nav2 integration.

## 1. Installation

### 1.0 Install ROS-based dependencies:
```bash
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-velodyne
sudo apt install ros-humble-velodyne-gazebo-plugins
sudo apt-get install ros-humble-velodyne-description
```

### 1.1 Clone and install all dependencies:
    
```bash
sudo apt install -y python3-rosdep
rosdep update

mkdir -p go2_ws/src
cd go2_ws/src
git clone https://github.com/widegonz/unitree-go2-ros2.git
cd ~/go2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 1.2 Build your workspace:
```bash
cd ~/go2_ws
colcon build
. go2_ws/install/setup.bash
```
## 2. Quick Start

You don't need a physical robot to run the following demos. Make sure you have ros2_control, gazebo ros, controller manager packages installed in your ros2 setup.

### 2.1 Gazebo demo: Run the Gazebo environment
```bash
ros2 launch go2_config gazebo.launch.py
```
![Go2 Gazebo Launch](.docs/gazebo_launch.png)

### 2.2 Walking demo in RVIZ: Run the gazebo along with rviz
```bash
ros2 launch go2_config gazebo.launch.py rviz:=true
```
![Go2 Gazebo RViz Launch](.docs/gazebo_rviz_launch.png)

### 2.3 Run the teleop node:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
https://github.com/user-attachments/assets/bcfeec70-12c5-49b8-b7a7-da4fa9b6dea5

### 2.4 Go2 Velodyne Config Gazebo demo: Run the Gazebo environment
```bash
ros2 launch go2_config gazebo_velodyne.launch.py 
```
![Go2 Velodyne Gazebo Launch](.docs/gazebo_velodyne_launch.png)

### 2.5 Go2 Veldyne Config Walking/PointCloud demo in RVIZ: Run the gazebo along with rviz
```bash
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true
```

> Note: set point cloud topic to `/velodyne_points`

![Go2 Velodyne Gazebo RViz Launch](.docs/gazebo_velodyne_rviz_launch.png)

### 2.6 Go2 Hokoyu 2D LiDAR Config Gazbeo demo: Run the Gazebo environment

> NOTE: To use Laser instead of 3D Velodyne LiDAR, comment `<xacro:include filename="$(find go2_description)/xacro/velodyne.xacro"/>` and uncomment `<xacro:include filename="$(find go2_description)/xacro/laser.xacro"/>` in `robot_VLP.xacro` file located inside `robots/description/go2_description/xacro/` folder.

```bash
ros2 launch go2_config gazebo_velodyne.launch.py 
```

To Run the gazebo along with rviz
```bash
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true
```

## 3. Tuning gait parameters

The gait configuration for your robot can be found in <my_robot_config>/gait/gait.yaml.

![CHAMP Setup Assistant](https://raw.githubusercontent.com/chvmp/champ_setup_assistant/master/docs/images/gait_parameters.png)

- **Knee Orientation** - How the knees should be bent. You can can configure the robot to follow the following orientation .>> .>< .<< .<> where dot is the front side of the robot.

- **Max Linear Velocity X** (meters/second) - Robot's maximum forward/reverse speed.

- **Max Linear Velocity Y** (meteres/second) - Robot's maximum speed when moving sideways.

- **Max Angular Velocity Z** (radians/second)- Robot's maximum rotational speed.

- **Stance Duration** (seconds)- How long should each leg spend on the ground while walking. You can set this to default(0.25) if you're not sure. The higher the stance duration the further the displacement is from the reference point.

- **Leg Swing Height** (meters)- Trajectory height during swing phase.

- **Leg Stance Height** (meters)- Trajectory depth during stance phase.

- **Robot Walking Height** (meters) - Distance from hip to the ground while walking. Take note that setting this parameter too high can get your robot unstable.

- **CoM X Translation** (meters) - You can use this parameter to move the reference point in the X axis. This is useful when you want to compensate for the weight if the center of mass is not in the middle of the robot (from front hip to rear hip). For instance, if you find that the robot is heavier at the back, you'll set a negative value to shift the reference point to the back.

- **Odometry Scaler** - You can use this parameter as a multiplier to the calculated velocities for dead reckoning. This can be useful to compensate odometry errors on open-loop systems. Normally this value ranges from 1.0 to 1.20.

## 4. Changing the World
It is important to note that the first time the simulation is launched, Gazebo will take a considerable amount of time to load the world. This is normal behavior and occurs because the software must download the 3D models from the internet to save them in the computer's cache, as well as compile the graphic resources and process the physical structures of the environment for the first time. In subsequent runs, startup will be much faster as all these files will be ready and stored locally.

There are a total of 7 worlds that we can select for simulation with our robot:
- bookstore
![bookstore](https://github.com/user-attachments/assets/b4215f66-ada5-45d4-9579-5e00cd1b5327)
- factory
![factory](https://github.com/user-attachments/assets/c504fa95-99aa-4d08-9307-a83a2819d432)
- office
![office](https://github.com/user-attachments/assets/26195ff4-16c9-4f4f-ad45-7397f3be00df)
- small_house
![small_house](https://github.com/user-attachments/assets/bbaef9cc-d461-42ba-87a8-7986d0754598)
- default
- outdoor
- playground

To use these worlds, we must do the following:

1. Export the Gazebo-Path
```bash
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/israel/go2_ws/src/unitree-go2-ros2/robots/configs/go2_config/models/' >> ~/.bashrc
```

Change the `<your_user>` part to your username.

2. Include the `world` variable in either of the 2 previous launch files:
```bash
ros2 launch go2_config gazebo.launch.py world:=<world_name>
```
If we want to use the bookstore world, we change `<world_name>` to `bookstore`:
```bash
ros2 launch go2_config gazebo.launch.py world:=bookstore
```

If we want to use the LiDAR and RViz:
```bash
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true world:=bookstore
```
With this, we can now use any of the worlds defined within the simulator.

## 5. Running the SLAM_Toolbox Package for Map Generation

1. Launch the simulator with the desired world and LiDAR enabled:
```bash
ros2 launch go2_config gazebo_velodyne.launch.py world:=bookstore
```

2. Execute the launch file that utilizes the `slam_toolbox` package:
```bash
ros2 launch go2_config slam.launch.py use_sim_time:=true
```

The RViz panel will open as shown below:
<img width="1850" height="1053" alt="Screenshot from 2025-12-22 22-11-50" src="https://github.com/user-attachments/assets/b01bac03-87a5-4777-ba94-e691a3091d9c" />


3. Run the teleoperation node:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

4. Navigate the robot:
  - It is crucial to mention that during the mapping process, the robot must not collide with any obstacles. This is primarily because a collision disrupts the odometry, causing the LiDAR data to mismatch with the measurements, which results in a corrupted map.

5. Save the map using the following command:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

  - This command will generate the `.pgm` and `.yaml` files for the map in the root directory. Once this map is generated, trajectory planning can be performed.


## Acknowledgements

This project builds upon and incorporates work from the following projects:

* [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros) - For the Go2 robot description (URDF model).
* [CHAMP](https://github.com/chvmp/champ) - For the quadruped controller framework.
* [CHAMP Robots](https://github.com/chvmp/robots) - For robot configurations and setup examples.

We are grateful to the developers and contributors of these projects for their valuable work.
