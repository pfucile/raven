
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
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/robot_state/conversions.h>




// Declaration of the global parameter from raven_code.cpp
extern float max_print_speed;
extern float print_temp;
extern float nozzle_diameter;
extern float extrusion_multiplier;
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
extern float** myArray;

extern serial::Serial ser;
extern EigenSTL::vector_Isometry3d pattern_poses;
extern descartes_planner::DensePlanner* planner;
extern ros::Publisher goal_pub;
extern ros::Publisher print_stat;
extern ros::Publisher trajectory_pub;





//the classes used

class segment_wise_printer_class {
private:
    const std::string PLANNING_GROUP = "xarm7";
    const std::string robot_description = "robot_description";
    const std::string world_frame = "world";
    const std::string tcp_frame = "extruder_tip";
    std::string follow_joint_trajectory_action = "/xarm/xarm7_traj_controller/follow_joint_trajectory";
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group;
    std::vector<std::vector<float>> GcodeArray;
    float feed_rate_for_GcodeArray = 0.0;
    float Extrusion_for_GcodeArray = 0.0;
    float time_for_GcodeArray  = 0.0;
    int seq_element_num;
    int i;
    std::vector<int> dataArray; // Array to store processed data
    std::vector<double> joint_group_positions_ss;
    std::vector<descartes_core::TrajectoryPtPtr> result;
    geometry_msgs::PoseStamped current_pose;
    std::vector<double> joint_pose;
    Eigen::Isometry3d pattern_origin;
    descartes_core::TrajectoryPtPtr pt ;
    Eigen::Isometry3d pose;
    trajectory_msgs::JointTrajectory joint_solution;
    float** myArray; // Pointer to a 2D array
    int rows;
    int cols;
    float print_velocity;
    std::vector<descartes_core::TrajectoryPtPtr> plan_result;
    descartes_planner::DensePlanner* planner;
    //descartes_core::RobotModelPtr model;
    //calculating the value for adjusting the starting point of print to the zero position that we have designated
    float ori_adj_x ;
    float ori_adj_y ;
    float ori_adj_z ;
public:
    segment_wise_printer_class(const std::string& planning_group_name);  // Constructor
    void setArray(float** arr, int r, int c);  // member functions
    void intialize_for_printing();
    void get_current_joint_position();
    void set_start_and_stop(const std::vector<int>& input);
    void spiral_move_down(float time_to_start);
    void direct_move_to_point(float time_to_start);
    std::vector<std::vector<float>> processSegment(std::string starting_style,std::string ending_style, float retract_distance);
    std::vector<std::vector<float>> init_point();
    trajectory_msgs::JointTrajectory path_planner();
    void print(trajectory_msgs::JointTrajectory joint_solution_to_print , std::vector<std::vector<float>> Gcode_array_of_segment);
    void update_starting_joint_pose(trajectory_msgs::JointTrajectory& joint_solution_to_print);
    std::vector<std::vector<float>> Go_to_joint_pose(std::vector<double> target_joint_pose,double time_in_seconds);
    std::vector<std::vector<float>> Go_to_cartesian_pose(std::vector<double> target_pose, double time_in_seconds);
    std::vector<double> find_pose_forward_kinematics(std::vector<double> joint_pose);
    std::vector<std::vector<float>> Follow_joint_pose_path_with_descartes(std::vector<std::vector<double>>  joint_path, float speed);
    

};



//function definitions

std::vector<descartes_core::TrajectoryPtPtr> makePath();
bool execute_gcode_sequence_by_sequence(float ori_adj_x, float ori_adj_y, float ori_adj_z);
bool include_objects_to_env();
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);
int sendGcode(std::vector<std::vector<float>>& GcodeArray_to_print );
int testExtrusionCalculation(std::vector<std::vector<float>>& GcodeArray_to_print );
float calculate_time_for_E_and_F(float E, float F );
float calculate_F_for_E_and_time(float Time, float E );
std::vector<descartes_core::TrajectoryPtPtr> go_to_point(std::vector<double>, double time_to_point);
int publishGoal(float Rx,float Ry,float Rz,float TBP,float Px,float Py,float Pz );
int Calculate_file_length(std::string path_to_file);
int Write_Gcode_to_Array(std::string path_to_file);
bool initialize_extruder(float print_temp);
bool set_temperature(float print_temp);
std::vector<float>  Calculate_origin_adjustment(float move_down_value, float** myArray);
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Isometry3d& pose, double dt);
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt);
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory, std::string follow_joint_trajectory_action );
bool attach_collision_objects(float Position_x,float Position_y,float Position_z,float Orientation_w,float Dimension_x,float Dimension_y,float Dimension_z,std::string name,std::string Frame );
bool include_collision_objects_to_env(float Position_x,float Position_y,float Position_z,float Orientation_w,float Dimension_x,float Dimension_y,float Dimension_z,std::string name );
std::vector<std::vector<int>> find_segments(float** myArray);
bool Move_to_pose_with_moveit_cartesian(double X,double Y,double Z,double Rx,double Ry,double Rz);
bool Move_to_pose_with_moveit(double X,double Y,double Z,double Rx,double Ry,double Rz);
bool display_planned_path(trajectory_msgs::JointTrajectory& joint_solution_to_print);




#endif // RAVEN_RAVEN_CODE_FUNCTIONS_H
