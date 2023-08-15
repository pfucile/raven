# RAVEN Robotic-Bioprinting-3D-Mentor
The repository of the codes for controlling the sawyer robot using ROS, Moveit and other related packages. The ROS workspace used to manage the robot is available here this can be cloned into your computer. Follow the procedures given in the SOP folder to set up all the necessary software and dependencies for getting the workspace up and running. 



## Raven Installation - Steps to follow 
### Create a catkin workspace according to the procedure for your robot
  
### Clone the RAVEN package to the catkin workspace
git clone https://github.com/vivekcdavid/raven.git

### Create the ikfast-plugin for your robot

Follow this instructions for creating the plugin 
https://ros-planning.github.io/moveit_tutorials/doc/ikfast/ikfast_tutorial.html#tweaking-the-creation-process
http://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/ikfast_tutorial.html
Old method , but explains clearly
http://docs.ros.org/en/hydro/api/moveit_ikfast/html/doc/ikfast_tutorial.html
https://choreo.readthedocs.io/en/latest/doc/ikfast_tutorial.html

follow this link to understand about ikfast plugins
http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types 


While generating the plugin use the _transform6d_ type
### Now we need to setup Descartes in the workspace
Before installing the Descartes package some dependencies need to be installed. 

Installing Orocos Kinematics and Dynamics package 

[https://www.orocos.org/wiki/Installation_Manual.html](https://www.orocos.org/wiki/Installation_Manual.html) //link to follow for installation of package 

[https://wiki.ros.org/ROS/Tutorials/BuildingPackages](https://wiki.ros.org/ROS/Tutorials/BuildingPackages) //link for using cmake  

Steps 

1. Go to the workspace and source the workspace 
    

sudo apt  install cmake-curses-gui 

cd ~/ws_moveit/ 

source /opt/ros/noetic/setup.bash 

2. Clone the package into the workspace 
    

git clone [https://github.com/orocos/orocos_kinematics_dynamics.git](https://github.com/orocos/orocos_kinematics_dynamics.git) 

3. Go into the folder named orocos_kdl 
    

cd orocos_kinematics_dynamics/orocos_kdl 

4. Make a directory named build inside this folder and go into the folder 
    

mkdir build 

cd build 

5. Build this package using the cmake 
    

ccmake .. 

In the window that open up, press c for configuring then go down to the CMAKE_INSTALL_PREFIX option and then press enter then replace the path with “/opt/ros/noetic”. Then press enter to confirm. Then press c to configure. Then press g to generate the files. 

Then run the following commands in the terminal 

make 

make check 

sudo make install 

Installing Descartes from source; 

[http://wiki.ros.org/descartes/Tutorials/Getting%20Started%20with%20Descartes](http://wiki.ros.org/descartes/Tutorials/Getting%20Started%20with%20Descartes) //source to follow 

Steps 

6. Clone the Descartes repository into the src folder of the workspace; 
    

cd ~/ws_moveit 

catkin_make 

source /opt/ros/noetic/setup.bash 

cd ~/ws_moveit/src 

 git clone https://github.com/ros-industrial-consortium/descartes.git 

 cd ~/ws_moveit/src  

rosdep install -r -y --from-paths src --ignore-src 

 catkin_make 

 source devel/setup.bash 

//wile building with the sawyer_ros/moveit/simulator setup some errors were returned, so the error causing package dependencies such as the joystick was removed from the respective package.xml file and then this command ran without error, the effects of this action is unknown!! 

******* to be tested check the dependencies of xarm and see if it could solve the error for the joystick and all 

7. Configure the build workspace 
    

catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release 

catkin_make 

// if you get the following error “ catkin: command not found” run the following comment 

sudo apt-get install python3-catkin-tools 

8. If an error related to “moveit_resources” appear; 
    

[https://github.com/PickNikRobotics/descartes_capability/issues/9](https://github.com/PickNikRobotics/descartes_capability/issues/9) //About the error   

[https://github.com/fzoric8/descartes_capability/commit/b7ac700fa073d1210e44da3308413dbb9e4dc65b](https://github.com/fzoric8/descartes_capability/commit/b7ac700fa073d1210e44da3308413dbb9e4dc65b)  

Open the file CMakeLists.txt file in the folder ws_moveit/src/descartes_capability and comment out the lines containing moveit_resources (lines 136 and 137) with “#”. 

Then run the commands in the step 2 again 

9. Source the workspace 
    

source devel/setup.bash 

To use the Descartes capability with Moveit follow the instructions given in the following link 

[https://github.com/PickNikRobotics/descartes_capability](https://github.com/PickNikRobotics/descartes_capability) 

To generate IKFast plugin 

[https://ros-planning.github.io/moveit_tutorials/doc/ikfast/ikfast_tutorial.html#tweaking-the-creation-process](https://ros-planning.github.io/moveit_tutorials/doc/ikfast/ikfast_tutorial.html#tweaking-the-creation-process) 

Now we need to copy the IKFast plugging we generated into the src folder of the workspace and then run the “catkin_make” command. 

To use Descartes we need to edit the kinematics.yaml file and change the kinematics solver from “kdl_kinematics_plugin/KDLKinematicsPlugin” to “sawyer_right_arm/IKFastKinematicsPlugin” 

Now if we use moveit cartesian planning it should automatically use the Descartes planning capability. 

If the planning is not working in edit the CmakeLists.txt and package.xml files in the ws_moveit/src/descartes_capability folder and include “eigen_conversions” in the depends on list 

In package.xml add the following line at line 26; 

<depend>eigen_conversions</depend> 

In CmakeLists.txt add the following line in line 23; 

eigen_conversions

### Edit the bash file for launching the RAVEN according to the procedure for your robot
* use the bash script templates available in the scripts folder for your reference 
### edit the Group names and other particulars in the raven file(src/raven_code.cpp) according to the specifications of the robot
* change the PLANNING_GROUP to the group name used in moveit
* change tcp_frame to the eff name being used
* change base_farem to the base name being used 
* set the FollowJointTrajectoryAction topic based on the rostopics being published by the robot
* change the endeffector in position_publisher and sub_plotter
