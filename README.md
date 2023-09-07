# RAVEN Robotic Bioprinting 3D Mentor
RAVEN is a robotic printing package aimed at accelerating robotic bioprinting technology development. The package makes it possible to convert a robot arm with six or seven degrees of freedom (6DOF or 7DOF) into a 3D printing  system. RAVEN handles the path planning for the robot and enables the simultaneous control of an extruder which is connected to the workstation via a USB connection. We are using Descartes Cartesian planner for trajectory planning. To improve the performance of the Descartes planner we are using IKFast Kinematic plugin for the Inverse kinematics this will have to be generated for your robot as explained in the instructions below. The package is capable of simulating the motion of the robot in Gazebo if needed, which is useful while working with complicated trajectories for the first time. 

The code in the repository assumes that you are using the xARM7 robot from UFactory. Instructions to edit the files according to your robot are also explained in the RAVEN Installation instructions section. Follow the procedures given in the instructions below, to set up all the necessary software and dependencies for getting the workspace up and running. We hope that this system would enable the printing of more advanced scaffold structures helping improve the tissue engineering field. 
## Disclaimer
___This code has only been tested for a few robotic systems. The code is currently in an experimental state, feel free to use this code for your projects and to contribute back to this project. But exercise caution while running the codes on real robots especially while testing the system on your robot for the first time.___ 
## System requirements
The hardware of the workstation should be capable enough to run Gazebo simulations and handle communications with the robot. Check the requirements of the [Gazebo](https://classic.gazebosim.org/tutorials?cat=guided_b&tut=guided_b1) and requirements for your robot for more details. This package works with [ROS Noetic](http://wiki.ros.org/noetic) running on [Ubuntu 20.04](https://releases.ubuntu.com/20.04.6/). For the RAVEN system to work properly the extruder should be attached to the robot such that it is collinear with the Z-axis of the end-effector of the robot. The extruder should be controlled by a standard FDM printer control board which in turn is connected to the workstation via a USB connection. For communications with the extruder via USB the USER should be added to the dialout group by running ```sudo usermod -aG dialout yourusername```.
## RAVEN Installation instructions - Steps to follow 
### Create a catkin workspace according to the procedure for your robot
  You should follow the instructions specifically for your robot. Please Refer to the instructions from the manufacturer of your robot manufacturer
  In case you need to create a generic ros workspace follow the tutorials from [ROS](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html ) and [Moveit](https://moveit.ros.org/install/ ). All the following instructions assume that the workspace is created in the home folder of the OS and it is named "ws_moveit".

### Modifying the URDF to include the extruder
Usually, commercial robotic arms will have a gripper or some tool that can be attached to the robot. Usually, the manufacturer would provide the URD for this tool. We can make use of this basic structure and create a URDF for our tool so that it can be attached to our robot. The path to the CAD models would also have to be given in the URDF. Follow the [tutorials for URDF](http://wiki.ros.org/urdf/Tutorials) to understand how this can be done, also please refer to the documentation of the manufacturer of the robot to for further details.

an example of URDF used to define an extruder is given here.
 ```
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="extruder">
  <xacro:macro name="extruder_urdf" params="prefix:='' attach_to:='' attach_xyz:='0 0 0' attach_rpy:='0 0 0' ">
    <xacro:unless value="${attach_to == ''}">
      <joint name="${prefix}extruder_support_joint" type="fixed">
        <parent link="${attach_to}"/>
        <child link="${prefix}extruder_support"/>
        <origin xyz="${attach_xyz}" rpy="${attach_rpy}"/>
      </joint>
    </xacro:unless>
    <link name="${prefix}extruder_support">
      <xacro:common_link_visual 
        mesh_filename="Extruder/Support_v3_scaled.STL"
        origin_xyz="-0.0382 -0.054 0" 
        origin_rpy="0 0 0"
        material_name="${prefix}White" />
      <xacro:common_link_collision 
        mesh_filename="Extruder/Support_v3_scaled.STL"  
        origin_xyz="-0.0382 -0.054 0" 
        origin_rpy="0 0 0" />
      <inertial>
        <origin rpy="0 0 0" xyz="-0.00033 -0.025 0.025"/>
        <mass value="0.115"/>
        <inertia ixx="0.00018075" ixy="0.000000999" ixz="-0.0000001" iyy="0.00019342" iyz="-0.00002871" izz="0.0001586"/>
      </inertial>
    </link>
    <joint name="${prefix}extruder_attachment" type="fixed">
      <origin xyz="0.0935 0 0.0535" rpy="0 0 0" />
      <axis xyz="0 0 1"/> 
      <parent link="${prefix}extruder_support" />
      <child link="${prefix}extruder_body" />
    </joint>
    <link name="${prefix}extruder_body">
      <xacro:common_link_visual 
        mesh_filename="Extruder/E3D_Hemera_Scaled.STL"
        origin_xyz="-0.0925 -0.025 0.0175" 
        origin_rpy="0 0 0"
        material_name="${prefix}White" />
      <xacro:common_link_collision 
        mesh_filename="Extruder/E3D_Hemera_Scaled.STL"  
        origin_xyz="-0.0925 -0.025 0.0175" 
        origin_rpy="0 0 0" />
      <inertial>
        <origin rpy="0 0 0" xyz="-0.0000158 0.0157447 0.0206782"/>
        <mass value="0.388"/>
        <inertia ixx="0.0001648" ixy="0.0000003" ixz="0.0000007" iyy="0.0001217" iyz="-0.0000008" izz="0.0001603"/>
      </inertial>
    </link>
    <link name="${prefix}extruder_tip" />
    <joint name="${prefix}extruder_tcp" type="fixed">
      <origin xyz="0.094 0 0.0535" rpy="0 0 0" />
      <axis xyz="0 0 1"/> 
      <parent link="${prefix}extruder_body" />
      <child link="${prefix}extruder_tip" />
    </joint>
  </xacro:macro>
</robot>
 ```

After editing the URDF we need to reconfigure the Moveit Configuration files accordingly. This can be done using the [MoveIt Setup Assistant](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html).
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
Make sure everything is aligned in the preview of the robot. Also, change the end-effector to the tip of the extruder and then generate the new Moveit configuration.   
### TCP Calibration
To calibrate the TCP (i.e. the tip of the extruder) in the URDF (to make sure the model defined in URDF is the same as the actual extruder on the real robot) we can use the [TCP Calibration package](https://github.com/Jmeyer1292/tool_point_calibration). Follow the instructions on the website to setup and use the package, based on the results edit the URDF. Use the following commands to install the package and its dependencies;

First, we need to install the ceres-solve and its dependencies.
``` # CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev
```
Now we need to clone the ceres-solve package and make.
```
cd ~
wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz
tar zxf ceres-solver-2.1.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.1.0
make -j3
make test
# Optionally install Ceres, it can also be exported using CMake which
# allows Ceres to be used without requiring installation, see the documentation
# for the EXPORT_BUILD_DIR option for more information.
make install
```
Now we need to clone the TCP calibration package into the src folder of our ROS workspace.
```
cd ~/ws_mveit/src
git clone https://github.com/vivekcdavid/tool_point_calibration.git
cd ..
source devel/setup.bash
catkin_make
```
Now follow the instructions from the [TCP Calibration package website](https://github.com/Jmeyer1292/tool_point_calibration) to do the calibration.

### Create the ikfast-plugin for your robot
To generate IKFast plugin, follow the instructions in the following tutorial from ["Moveit"](https://ros-planning.github.io/moveit_tutorials/doc/ikfast/ikfast_tutorial.html). This method relies on a docker container for [Openrave](http://openrave.org/) which makes it straightforward to create the plugin. But if you are interested ins understanding the exact process follow the tutorial for [Moveit Indigo](http://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/ikfast_tutorial.html) or [Moveit Hydro](http://docs.ros.org/en/hydro/api/moveit_ikfast/html/doc/ikfast_tutorial.html) or [this tutorial](https://choreo.readthedocs.io/en/latest/doc/ikfast_tutorial.html), which explains the process clearly.

While generating the plugin use the [transform6d ](http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types) type.Now we need to copy the IKFast plugging we generated into the src folder of the workspace and then run the “catkin_make” command. To use Descartes we need to edit the kinematics.yaml file and change the kinematics solver from ```“kdl_kinematics_plugin/KDLKinematicsPlugin”``` to ```“<myrobot_name>_<planning_group_name>_kinematics/IKFastKinematicsPlugin”```.
  
### Now we need to setup Descartes in the workspace
  Before installing the Descartes package some dependencies need to be installed. 
  * Installing Orocos Kinematics and Dynamics [package](https://www.orocos.org/wiki/Installation_Manual.html).
Go to the workspace and source the workspace 
  ```
  sudo apt  install cmake-curses-gui 
  cd ~/ws_moveit/ 
  source /opt/ros/noetic/setup.bash
  ```
  Clone the package into the workspace 
  ```
  git clone https://github.com/orocos/orocos_kinematics_dynamics.git
  ```
  Go into the folder named orocos_kdl
  ```
  cd orocos_kinematics_dynamics/orocos_kdl 
  ```
  Make a directory named build inside this folder and go into the folder 
   ```   
  mkdir build 
  cd build 
  ```
  Build this package using the cmake 
  ```
  ccmake .. 
  ```
  In the window that open up, press c for configuring then go down to the CMAKE_INSTALL_PREFIX option and then press enter then replace the path with “/opt/ros/noetic”. Then press enter to confirm. Then press c to configure. Then press g to generate the files. 
  Then run the following commands in the terminal 
  ```
  make
  sudo make install
  ```
  * Installing [Descartes](http://wiki.ros.org/descartes/Tutorials/Getting%20Started%20with%20Descartes) from source;

  Clone the Descartes repository into the src folder of the workspace; 
  ```
  cd ~/ws_moveit 
  catkin_make 
  source /opt/ros/noetic/setup.bash 
  cd ~/ws_moveit/src 
  git clone https://github.com/ros-industrial-consortium/descartes.git 
  cd ~/ws_moveit 
  rosdep install -r -y --from-paths src --ignore-src 
  catkin_make 
  source devel/setup.bash
   ```
Configure the build workspace 
      
  ```
  catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release 
  catkin_make 
  ```
Source the workspace 
  ```
source devel/setup.bash
```
### Installing the "serial" package for communicating with the extruder
  This [package](http://wjwwood.io/serial/ ) is used for communication with the extruders' controller board. The instructions given below are based on the instructions from the 
  [website](http://wjwwood.io/serial/doc/1.1.0/index.html).
 ```
  source /opt/ros/noetic/setup.bash 
  cd ~/ws_moveit/src
  git clone https://github.com/wjwwood/serial.git
  cd serial 
  make 
  make test 
  make doc  
  sudo make install
  cd ~/ws_moveit/
  catkin_make 
  ```
Don't forget to add the user to the dialout group by running ports```sudo usermod -aG dialout $USER```, so that RAVEN can access the serial 
### Clone the RAVEN package to the catkin workspace
 ```
  git clone https://github.com/vivekcdavid/raven.git
  ```
Now build the workspace again
 ```
  cd ~/ws_moveit
  source /opt/ros/noetic/setup.bash 
  catkin_make 
  ```
Now source ros and the workspace again
 ```
  cd ~/ws_moveit
  source /opt/ros/noetic/setup.bash 
  source devel/setup.bash
  ```
### Creating and running bash script for launching RAVEN
Depending on the robot there will be changes in the exact code which has to be executed for running the printing system. But the sequence of steps that should be taken is as follows;
* The robot(simulated/real) should be connected with the workstation
* Then Movit should be launched and it should be able to control the robot
* Then it should be ensured that the joint_states should be avilable from ```robot/joint_states```. This can be done by running ```rostopic list``` to see all the available topics and you can use ```rostopic echo <topic_name>``` to see what is being published in a topic. If needed some ros topics will have to be relayed.
* The ```ikfast_base_frame``` and ```ikfast_tool_frame``` should be set in rosparam server.
* Then the data logging function of RAVEN can be started by running ```rosrun raven Position_publisher.py```  and ```rosrun raven Data_logger.py```.
* Then the RAVEN can be laucnhed by running ```roslaunch raven raven_launch_file.launch```

Use the bash script templates available in the scripts folder (ws_moveit/src/raven/scripts) for your reference.
To run this bash script we need to set up another script that handles running commands along different terminals. To run the simulation or a script for the real robot, multiple commands have to be executed on multiple terminals in Ubuntu. This Bash script helps us to automate this process. This file uses a script to automatically send our commands to the designated terminal window. More details can be found in [this links](https://askubuntu.com/questions/641683/how-can-i-send-commands-to-specific-terminal-windows). In order for this script to work please follow the instructions given below. 

install wmctrl and xdotool: 
     ```
    sudo apt-get install wmctrl xdotool 
     ```
Create a file named target_term in the /bin folder using the following command;
```
cd /bin
sudo nano target_term
```
and copy paste the following code into it and save the file; 
 ```
#!/usr/bin/env python3
import subprocess
import os
import sys
import time
#--- set your terminal below
application = "gnome-terminal"
#---

option = sys.argv[1]
data = os.environ["HOME"]+"/.term_list"

def current_windows():
    w_list = subprocess.check_output(["wmctrl", "-lp"]).decode("utf-8")
    w_lines = [l for l in w_list.splitlines()]
    try:
        pid = subprocess.check_output(["pgrep", application]).decode("utf-8").strip()
        return [l for l in w_lines if str(pid) in l]
    except subprocess.CalledProcessError:
        return []

def arr_windows(n):
    w_count1 = current_windows()
    for requested in range(n):
        subprocess.Popen([application])
    called = []
    while len(called) < n:
        time.sleep(1)
        w_count2 = current_windows()
        add = [w for w in w_count2 if not w in w_count1]
        [called.append(w.split()[0]) for w in add if not w in called]
        w_count1 = w_count2

    return called

def run_intterm(w, command):
    subprocess.call(["xdotool", "windowfocus", "--sync", w])
    subprocess.call(["xdotool", "type", command+"\n"]) 

if option == "-set":
    open(data, "w").write("")
    n = int(sys.argv[2])
    new = arr_windows(n)
    for w in new:
        open(data, "a").write(w+"\n")
elif option == "-run":
    t_term = open(data).read().splitlines()[int(sys.argv[2])-1]
    command = (" ").join(sys.argv[3:])
    run_intterm(t_term, command)
 ```
then run the following command and make the file executable;
```
cd /bin
sudo chmod +x target_term
```

### Editing RAVEN file(ws_moveit/src/raven/src/raven_code.cpp) according to the specifications of the robot
  * Edit the " path_to_file" according to your folder structure
  * Change the PLANNING_GROUP to the group name used in moveit
  * Set the FollowJointTrajectoryAction topic based on the rostopics being published by the robot
  * Change tcp_frame to the name of the end-effector of the robot
  * Change world_frame to the name of the base link of your robot
 
Also, change the group_name name in the files Position_publisher.py and Data_logger.py according to your naming.

## Instructions for making a Gcode for RAVEN

* Each line in the Gcode file represents a point in the trajectory.
* Each line should have the following values ;
    * G - Either 0 or 1 representing movements without and with extrusion
    * F - Pives the speed of the motion to this point in m/min
    * X - Position in X axis in meters
    * Y - Position in Y axis in meters
    * Z - Position in Z axis in meters
    * E - The amount of filament to be extruded while moving to the point in millimeters
    * Rx - Rotation about X-axis of the extruder
    * Ry - Rotation about Y-axis of the extruder
    * Rz - Rotation about Z-axis of the extruder
    * seg# - seg# or segment number is used to segment the Gcode into multiple separate motion segments
*  By default in Moveit/Descartes the tool will be pointing up into the air.  But we want it to  
be pointing down into the ground, to make the robot point down the Rx value should be Pi (3.141592).
* The file shouldn't have any other lines with comments other than the first line.
* A point density of 4 points  per millimeter was found to give the best results, if the gap between the points are very large the trajectory generated by Descartes tend to  deviate from the ideal cartesian path.
* There shouldn't be any blank lines in the file.
* At the moment Rz values are not taken into consideration for path panning

#### Example 
  ```
  #G(0/1) F(m/min)      X(m)        Y(m)        Z(m)       E(mm)     Rx        Ry          Rz       seg# 
  1 0.6 0.002702 0.005 0.01175421 0.0 3.141592653589793 0.3490658503988659 0.0 0 
  1 0.3 0.002702 0.003 0.01175421 0.10449 3.141592653589793 0.3490658503988659 0.0 0 
  1 0.3 0.002702 0.001 0.01175421 0.20898 3.141592653589793 0.3490658503988659 0.0 0 
  1 0.3 0.002702 -0.001 0.01175421 0.31347 3.141592653589793 0.3490658503988659 0.0 0 
  1 0.3 0.002702 -0.003 0.01175421 0.41796 3.141592653589793 0.3490658503988659 0.0 0 
  1 0.3 0.002702 -0.005 0.01175421 0.52245 3.141592653589793 0.3490658503988659 0.0 0 
  ```

  
  If you have a standard Gcode created in Marlin flavour you can use the Gcode_converter.py file to convert to the RAVEN Gcode format
  for this paste the Gcode section with the motion into the to_convert.txt file and run the Gcode_converter.py file
  

## Instructions for printing 

  For printing the Gcode which is in the prescribed RAVEN format should be pasted in the converted_file.txt file.
  Then to print run  the RAVEN.bash to print on the real robot or RAVEN_Sim.bash to simulate the motion in Gazebo. ** Note that the extruder or a at least a dummy extruder(an Arduino connected via USB) is required for the Gazebo simulation to work ** 
  
  The first time that you are printing it is advised to put only two points and you should have access to the emergency stop at all times during the tests.
  To plot graphs you can use ```rosrun raven plotter.py```.

## Instructions for using an optimizer for optimizing your Gcode

  This simple optimization process makes it possible to use less reliable robotic arms also for 3D printing. It works by comparing the trajectory the robot actually followed with the path that was specified in the Gcode. Thus the deviation from the ideal trajectory is calculated and this is used to create a corrected Gcode file. This process is iterated 3 times to obtain the final 
  optimized Gcode.

#### !!!_This is an experimental code. You can use this code at your own risk_!!!
  To run this optimization process follow the instructions below.
  * Open a separate terminal and source the workspace
  * Run the command ```rosrun raven post_processing.py```
  * Then a dialogue box will open up asking you to select the original trajectory file, select the file here
  * Then another dialogue box will as you to select the log file which was created after as successful fun of the Gcode
  * The program will generate some graphs, you can close them after checking them.
  * After the graphs are closed, a new gcode file will be generated with the file name <original_trajectory_file>__processed_with_<log_file_name>_iteration_num_0.txt
  * Also another dialogue box would have opened asking for log file, without closing this, print using the new gcode and then select the log file generated.
  * This process would repeat for 2 more times.
  * And finally few graphs with the optimization results would be generated. after closing them the final code would be generated.

## Citing

  Please cite this work as:
