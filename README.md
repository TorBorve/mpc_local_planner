**MPC local planner**
=======
 
A nonlinear MPC used to control an autonomous car.
 
This repository contains an implementation of a nonlinear MPC that is used to control an autonomous car. To do this [ipopt](https://coin-or.github.io/Ipopt/) is used to solve a nonlinear optimization problem. [CppAD](https://coin-or.github.io/CppAD/doc/cppad.htm) is used to interface with ipopt. See this [example](https://www.coin-or.org/CppAD/Doc/ipopt_solve_get_started.cpp.htm) for more information.
 
The structure of the MPC is inspired by Udacity's [example MPC](https://medium.com/@techreigns/model-predictive-control-implementation-for-autonomous-vehicles-932c81598b49) for their simulator. On top of this there is implemented a ROS interface.
 
Dependencies
---------
---------
 
Currently the setup guide is not finished.
There will be an update soon with step by step instructions on how to install the required components.
 
ipopt
------
 
Ipopt is used as the solver for the optimization problem. Therefore it is necessary to install ipopt. Personally I had some problems doing this. I used this [tutorial](https://coin-or.github.io/Ipopt/INSTALL.html) and ran this [file](https://github.com/uppala75/CarND-MPC-Project/blob/master/install_ipopt.sh) from the Udacity MPC.
 
CppAD
------
 
CppAD has an interface to the ipopt solver that is used in the code. This can be installed by following this [guide](https://coin-or.github.io/CppAD/doc/install.htm). Could also be possible to use the command "sudo apt-get install cppad".
 
Build instructions
------------------
------------------
 
First we need to clone the repository into our source folder in a catkin workspace.
 
```terminal
cd ~/catkin_ws/src
git clone https://gitlab.stud.idi.ntnu.no/fuelfighter/autonomous/planning-control/mpc-local-planner.git
cd ..
catkin build ## (or catkin_make)
```
 
We are now ready to run the MPC. This is done by using launch files.
 
```terminal
source devel/setup.bash
roslaunch mpc_local_planner mpc.launch
```
 
References
----------
----------
 
- [ipopt](https://coin-or.github.io/Ipopt/)
- [example mpc](https://medium.com/@techreigns/model-predictive-control-implementation-for-autonomous-vehicles-932c81598b49)
- [CppAD](https://coin-or.github.io/CppAD/doc/cppad.htm)
