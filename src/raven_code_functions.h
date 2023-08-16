
#ifndef RAVEN_RAVEN_CODE_FUNCTIONS_H
#define RAVEN_RAVEN_CODE_FUNCTIONS_H


#include <ros/ros.h>
#include <serial/serial.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>
// Includes the descartes robot model we will be using
#include <descartes_moveit/ikfast_moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>
// Includes the utility function for converting to trajectory_msgs::JointTrajectory's
#include <descartes_utilities/ros_conversions.h>
#include <iostream>
#include <fstream>
#include <string>
#include <bits/stdc++.h>
#include <cmath>
#include <ctime>
#include <future>
#include <thread>
#include <boost/thread.hpp>
#include <unistd.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>


// Declaration of the global parameter from raven_code.cpp
extern float max_print_speed;
extern float print_temp;
extern float nozzle_diameter;
extern float layer_height;
extern float filament_diameter;
extern float starting_point[];
extern float move_down_value;
extern float time_mov_to_start;
extern float time_to_go_back;
extern double time_between_points ;
extern float print_velocity;
extern int* count;
extern const double default_joint_vel;
extern const std::string PLANNING_GROUP;
extern std::vector <std::string> names;
extern std::vector <double> seed;
extern float GcodeArray[10000][3];
extern float** myArray;

extern serial::Serial ser;
extern EigenSTL::vector_Isometry3d pattern_poses;
extern descartes_planner::DensePlanner* planner;
extern ros::Publisher goal_pub;
extern ros::Publisher print_stat;



std::vector<descartes_core::TrajectoryPtPtr> makePath();
std::vector<descartes_core::TrajectoryPtPtr> makePath_init();
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Isometry3d& pose, double dt);
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt);
std::vector<float>  Calculate_origin_adjustment();
int Write_Gcode_to_Array(std::string path_to_file);
bool execute_gcode_sequence_by_sequence(float ori_adj_x, float ori_adj_y, float ori_adj_z);
bool include_objects_to_env();
int Calculate_file_length(std::string path_to_file);
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);
int publishGoal(float Rx,float Ry,float Rz,float TBP,float Px,float Py,float Pz );
int sendGcode(int seq_num  );


//function definitions

std::vector<descartes_core::TrajectoryPtPtr> makePath_init();
int publishGoal(float Rx,float Ry,float Rz,float TBP,float Px,float Py,float Pz );
int Calculate_file_length(std::string path_to_file);
int Write_Gcode_to_Array(std::string path_to_file);
bool initialize_extruder(float print_temp);
bool set_temperature(float print_temp);
std::vector<float>   Calculate_origin_adjustment(float move_down_value);
int sendGcode(int seq_num );
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Isometry3d& pose, double dt);
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt);
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory, std::string follow_joint_trajectory_action );
bool attach_collision_objects(float Position_x,float Position_y,float Position_z,float Orientation_w,float Dimension_x,float Dimension_y,float Dimension_z,std::string name,std::string Frame );
bool include_collision_objects_to_env(float Position_x,float Position_y,float Position_z,float Orientation_w,float Dimension_x,float Dimension_y,float Dimension_z,std::string name );





#endif // RAVEN_RAVEN_CODE_FUNCTIONS_H
