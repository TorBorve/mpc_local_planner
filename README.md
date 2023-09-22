# **MPC local planner**
A nonlinear MPC used to control an autonomous car.

## :book: **Table of Contents**
- [:notebook_with_decorative_cover: **Description**](#description)
  * [**ROS interface**](#ROS_interface)
- [:hammer_and_wrench: **Install**](#install)
- [:rocket: **Usage**](#usage)
  * [**Parameters**](#parameters)
  * [**Run**](#run)
- [:link: **References**](#references)

<!-- <small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small> -->


## :notebook_with_decorative_cover: **Description** <a name="description"></a>

This repository contains an implementation of a control system used for an autonomous car. The control system is utilizes nonlinear MPCs (NMPC) for controlling the car. The control system was originally made for the Norwegian student organization at NTNU, [Fuel Fighter](https://www.fuelfighter.no/). [Acados](https://docs.acados.org/) is used to get access to fast solvers tailored for NMPC. Acados python interface is then used to generate C code that we use in our C++ program. The control system is designed to follow a reference path/trajectory. The path is given as a nav_msgs::Path message. This contains a vector with points defining the path. To make the car follow the path we firstly interpolate a third order polynomial to fit a small section of the track in front of the car. Then we input this into the solver. The mpc is designed so that the car wants to stay on the track and keep a constant speed. There are several parameters that can be tuned so that the mpc works optimally. See [parameter section](#parameters)
 
<div align="center">
<img src="images/mpc_demo.gif" width="400">
<br>
<figcaption align="center">Demo of MPC using gazebo simulator and audibot car</figcaption>
</div>

### **ROS interface** <a name="ROS_interface"></a>

The mpc is made for ROS2 Humble. All of the inputs to the mpc and outputs are therefore sent using topics and transforms. The MPC needs to know the state of the car and where the track or goal is. Transforms are used to figure out where the car is. The code looks up the latest transform from a map frame to a car frame. How fast we are going is sent to us as a Twist message on a given topic. It can also take in a topic where the actual steering angle of the car is published. However, it is also possible to assume that the steering angle is equal to the previous steering command. This can be done by setting `use_actual_steering_topic=False` in the param file. Lastly the MPC needs to know where we want the car to drive. This is sent over the path topic as a path message.
 
The output of the MPC is a steering angle and throttle value. These values are published on two separate topics: */steering_cmd* and */throttle_cmd*. In addition to the commands sent to the car several path messages are also published. */global_path* is the reference track. This is where we want the car to drive. This is the green line seen in the gif. */local_path* is where the mpc is planning to drive. This is shown in red. Lastly we have */interpolated_path*. This is a interpolation of the */global_path* using a third order polynomial. The names of most of the frames and topics can be found and changed in the file */params/mpc.yaml*
 
<div align="center">
<img src="images/rosgraph.svg" width="700">
<br>
<figcaption align="center">ROS interface for MPC</figcaption>
</div>

## :hammer_and_wrench: **Install** <a name="install"></a>

The MPC requires several things to work. Mainly it is the library Acados and of course ROS.

1. **ROS**
  ROS is used to send and receive data. It can be installed by following [this]((http://wiki.ros.org/noetic/Installation)) tutorial. If you already have ROS installed you can skip this. ROS noetic is used, however other versions may also work.

2. **Clone**
 
  Clone this repository into the src folder in a workspace. Example:
 
  ```terminal
  cd ~/ros2_ws/src
  git clone https://gitlab.stud.idi.ntnu.no/fuelfighter/autonomous/planning-control/mpc_local_planner.git
  ```

3. **Acados**
 
  The **recommended** way to instal acados is to use the install script *install_acados.sh*. First we need to install some dependencies. Run the following:
 
  ```terminal
  sudo apt install python3-virtualenv python3-pip
  ```
 
  Then we can use the script. It requires the path to where you want to install acados. It can be used as follows:
 
  ```terminal
  bash install_acados.sh /path/to/install/location
  ```

  The installation was successful if a graph pops up showing the solution by an example mpc.
 
  You can also install it manually by doing the following steps. Firstly follow this [guide](https://docs.acados.org/installation/index.html) to download and build the acados library. **Choose the Cmake alternative**. Once that is done we need to set it up with the python interface. I recommend using a virtual environment for this (as do they). To make compiling this code easier make sure to create the virtual environment in *<acados_root>/build/*. If you do not do this the code will not compile. You will have to change CmakeLists.txt. Follow the steps in this [guide](https://docs.acados.org/python_interface/index.html). Remember to add the lines from step four into *.bashrc*
 
4. **MPC**
 
  Now we are ready to build the mpc.
 
  ```terminal
  colcon build
  ```
 
  You can also build only the mpc by adding mpc_local_planner to the command:
 
  ```terminal
  colcon build --packages-select mpc_local_planner
  ```
 
## :rocket: **Usage** <a name="usage"></a>

### **Parameters** <a name="parameters"></a>

Once everything is installed and built we are ready to define the parameters for the mpc. This is done using *rosparam*. The parameters are defined in a *.yaml* file in the folder *config*. These parameters need to be correct for your vehicle so that the mpc will work optimally. If some critical parameters are not defined the code will not run. It will throw an error telling you what parameter it is missing. Other noncritical parameters cause warnings and some do not even do that. Therefore make sure your parameters are correct. The way the build system works makes it so that you need to recompile every time you update any of the parameters. Otherwise the changes will have no effect.
 
### **Run** <a name="run"></a>

Finally we are ready to run the code. This is done by using *launch* files. The main launch file is *mpc.launch.py*. You can run this file in the following way.

```terminal
source install/setup.bash
ros2 launch mpc_local_planner mpc.launch.py
```

It is also possible to only start rviz be using *rviz.launch*. The syntax is the same as for *mpc.launch*.

```terminal
ros2 launch mpc_local_planner rviz.launch.py
```
 
## :link: **References** <a name="references"></a>

- [Acados documentation](https://docs.acados.org/)
- [Acados github](https://github.com/acados/acados)