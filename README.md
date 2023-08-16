# RAVEN Robotic-Bioprinting-3D-Mentor
RAVEN is a robotic printing package aimed at accelerating robotic bioprinting technology development. The package makes it possible to convert a robot arm with six or seven degrees of freedom (6DOF or 7DOF) into a 3D printing  system. RAVEN handles the path planning for the robot and enables the simultaneous control of an extruder which is connected to the workstation via a USB connection. We are using Descartes cartesian planner for trajectory planning. We hope that this system would enable the printing of more advanced scaffold structures helping improve the tissue engineering field. 

Follow the procedures given in the instructions below, to set up all the necessary software and dependencies for getting the workspace up and running. The code in the repository assumes that you are using the xARM7 robot from U_r****. Instructions to edit the files according to your robot are explained in the Raven Installation instructions section


## Raven Installation instructions - Steps to follow 
### Create a catkin workspace according to the procedure for your robot
  You should follow the instructions specifically for your robot. Please Refer to the instructions from the manufacturer of your robot manufacturer
  In case you need to create a generic ros workspace the following links would help you.
  * https://moveit.ros.org/install/ 
  * https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html 
### Clone the RAVEN package to the catkin workspace
 ```
  git clone https://github.com/vivekcdavid/raven.git
  ```

### Create the ikfast-plugin for your robot
   To generate IKFast plugin, follow the instructions in the following tutorial form moveit [http://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/ikfast_tutorial.html]("IKFast Tutorial"). This method relies on a docker container for[http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types ] (Openrave) which makes it staraightforward to create the plugin. But if you are interested ins understanding the exact process follow the tutorial for [http://docs.ros.org/en/hydro/api/moveit_ikfast/html/doc/ikfast_tutorial.html](Moveit Hydro) or [https://choreo.readthedocs.io/en/latest/doc/ikfast_tutorial.html](this tutorial), which explains the process clearly.

```
While generating the plugin use the _transform6d_ type
```

  Now we need to copy the IKFast plugging we generated into the src folder of the workspace and then run the “catkin_make” command. 
  To use Descartes we need to edit the kinematics.yaml file and change the kinematics solver from 
   ```
 From  “kdl_kinematics_plugin/KDLKinematicsPlugin” to “sawyer_right_arm/IKFastKinematicsPlugin”  
   ```
  
### Now we need to setup Descartes in the workspace
  Before installing the Descartes package some dependencies need to be installed. 
  
  Installing Orocos Kinematics and Dynamics package 
  
  [https://www.orocos.org/wiki/Installation_Manual.html](https://www.orocos.org/wiki/Installation_Manual.html) //link to follow for installation of package 
  
  [https://wiki.ros.org/ROS/Tutorials/BuildingPackages](https://wiki.ros.org/ROS/Tutorials/BuildingPackages) //link for using cmake  
  
  Steps 
  1. Go to the workspace and source the workspace 
  ```
  sudo apt  install cmake-curses-gui 
  cd ~/ws_moveit/ 
  source /opt/ros/noetic/setup.bash
  ```
  2. Clone the package into the workspace 
  ```
  git clone [https://github.com/orocos/orocos_kinematics_dynamics.git](https://github.com/orocos/orocos_kinematics_dynamics.git) 
  ```
  3. Go into the folder named orocos_kdl
  ```
  cd orocos_kinematics_dynamics/orocos_kdl 
  ```
  5. Make a directory named build inside this folder and go into the folder 
   ```   
  mkdir build 
  cd build 
  ```
  5. Build this package using the cmake 
  ```
  ccmake .. 
  ```
  In the window that open up, press c for configuring then go down to the CMAKE_INSTALL_PREFIX option and then press enter then replace the path with “/opt/ros/noetic”. Then press enter to confirm. Then press c to configure. Then press g to generate the files. 
  
  Then run the following commands in the terminal 
  ```
  make 
  make check 
  sudo make install
  ```
  
  Installing Descartes from source; 
  
  [http://wiki.ros.org/descartes/Tutorials/Getting%20Started%20with%20Descartes](http://wiki.ros.org/descartes/Tutorials/Getting%20Started%20with%20Descartes) //source to follow 
  
  Steps 
  
  6. Clone the Descartes repository into the src folder of the workspace; 
      
  ```
  cd ~/ws_moveit 
  
  catkin_make 
  
  source /opt/ros/noetic/setup.bash 
  
  cd ~/ws_moveit/src 
  
   git clone https://github.com/ros-industrial-consortium/descartes.git 
  
   cd ~/ws_moveit/src  
  
  rosdep install -r -y --from-paths src --ignore-src 
  
   catkin_make 
  
   source devel/setup.bash
   ```
  
  //wile building with the sawyer_ros/moveit/simulator setup some errors were returned, so the error causing package dependencies such as the joystick was removed from the respective package.xml file and then this command ran without error, the effects of this action is unknown!! 
  
  ******* to be tested check the dependencies of xarm and see if it could solve the error for the joystick and all 
  
  7. Configure the build workspace 
      
  ```
  catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release 
  
  catkin_make 
  ```
  // if you get the following error “ catkin: command not found” run the following comment 
  
  sudo apt-get install python3-catkin-tools 
  
  9. Source the workspace 
  ```
source devel/setup.bash
```
  

### Installing the "serial" package for communicating with the extruder
  http://wjwwood.io/serial/ 
  http://wjwwood.io/serial/doc/1.1.0/index.html //code to follow for installation of library.
  
   Do this in the src folder of the environment 
 ```
  source /opt/ros/noetic/setup.bash 
  cd src
  git clone https://github.com/wjwwood/serial 
  cd serial 
  make 
  make test # (optional) builds the example and tests, and runs the tests. 
  make doc  # (optional) builds _this_ documentation. 
  sudo make install 
  //go back to the src folder now 
  cd .. 
  //go back to the home directory of the catkin environment  
  cd .. 
  catkin_make 
  ```
  Now in the package that you are going to use the “serial” function include serial in CMakeList.txt and package.xml . You can follow the example in the following link; 
  https://github.com/garyservin/serial-example 
  To use the function follow the documentation in the following link :
  http://wjwwood.io/serial/doc/1.1.0/classserial_1_1_serial.html 
  OR
  Follow the example given in the following link; 
  https://github.com/garyservin/serial-example/blob/master/src/serial_example_node.cpp 
### Edit the bash file for launching the RAVEN according to the procedure for your robot
  * use the bash script templates available in the scripts folder for your reference 
### edit the Group names and other particulars in the raven file(src/raven_code.cpp) according to the specifications of the robot
  * change the PLANNING_GROUP to the group name used in moveit
  * change tcp_frame to the eff name being used
  * change base_farem to the base name being used 
  * set the FollowJointTrajectoryAction topic based on the rostopics being published by the robot
  * change the endeffector in position_publisher and sub_plotter

## Instructions from making a Gcode for RAVEN

  // set the orientation. By default, the tool will be pointing up into the air when we usually want it to  
  // be pointing down into the ground. so make the robot point down the Rx value should be Pi (3.14.......)
  The file shouldn't have any other lines as the first line with commnts

#### example 
```
  #G(0/1) F(m/min)      X(m)        Y(m)        Z(m)       E(mm)     Rx        Ry          Rz       seg# 
  1 0.6 0.002702 0.005 0.01175421 0.0 3.141592653589793 0.3490658503988659 0.0 0 
  1 0.3 0.002702 0.003 0.01175421 0.10449 3.141592653589793 0.3490658503988659 0.0 0 
  1 0.3 0.002702 0.001 0.01175421 0.20898 3.141592653589793 0.3490658503988659 0.0 0 
  1 0.3 0.002702 -0.001 0.01175421 0.31347 3.141592653589793 0.3490658503988659 0.0 0 
  1 0.3 0.002702 -0.003 0.01175421 0.41796 3.141592653589793 0.3490658503988659 0.0 0 
  1 0.3 0.002702 -0.005 0.01175421 0.52245 3.141592653589793 0.3490658503988659 0.0 0 
```

  
  If you have a standard gcode created in Marlin flavour you can use the Gcode_converter.py file to conver to the RAVEN Gcode format
  for this paste the gcode section with the motion into the to_convert.txt file and run the Gcode_converter.py file


## Instructions for printing 

  For printing the Gcode which is in the prescribed RAVEN format should be pasted in the converted_file.txt file.
  Then to print run  the RAVEN.bash to print on the real robot or RAVEN_Sim.bash to simulate the motion in gazebo.
  
  first time that you are printing it is advised to put only two points and you should have access to the emergency stop at all times during the tests
