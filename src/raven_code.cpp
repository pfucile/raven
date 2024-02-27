// CLASSES

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
#include "raven_code_functions.h"



//defining some values and printing parameterfloat
float max_print_speed = 2.5; //m/min to be used if the print speed is too high
float print_temp = 230; //printing temperature
float nozzle_diameter = 0.4; //mm
float layer_height = 0.3; //mm
float extrusion_multiplier = 0.6; //mm
float filament_diameter = 1.75; //mm
float starting_point[] = {0.45, 0.065, 0.1325}; //give the desired initial starting pose here!!
float move_down_value = 0.0715;  //adjust this number if the extruder is too close or too far from the print bed
float time_mov_to_start = 10 ; //time taken to move from the waiting point to the first point on the segment
float time_to_go_back = 5.0;   //time taken to move away after print
EigenSTL::vector_Isometry3d pattern_poses;
double time_between_points = 0.15;

int* count;
const double default_joint_vel = 0.05; // Define the default speed in rad/s. We define this if the time between points is not provide


// Create an object from the Serial class for the extruder
serial::Serial ser;

// Create objects and functions for the trajectory planning

ros::Publisher goal_pub;
ros::Publisher print_stat;



//Initialization of different functions
int Write_Gcode_to_Array(std::string path_to_file,float** myarray);//Initialization of the function to load the gcode to a array for the functions to use
; //Initialization of the function to inlude the extruder nad table to environment
bool attach_collision_objects(float Position_x,float Position_y,float Position_z,float Orientation_w,float Dimension_x,float Dimension_y,float Dimension_z,std::string name,std::string Frame );
bool include_collision_objects_to_env(float Position_x,float Position_y,float Position_z,float Orientation_w,float Dimension_x,float Dimension_y,float Dimension_z,std::string name );
int Calculate_file_length(std::string path_to_file); // Initialization of the function to calculate the length of the Gcode

bool initialize_extruder(float print_temp);
std::vector<std::vector<int>> find_segments(float** myArray); // Initialization of the function to find the relevant segments

//the gcode file
std::string path_to_file = "../xarm_ws/src/raven/scripts/Converted_file.txt";


// Defining again right arm group, because move_group_interface from
// MoveIt and Descartes need different initialization types
// definition of the robot variables.

const std::string PLANNING_GROUP = "xarm7"; // Attribution of planning group variable


float** Original_gcode_array; // Initializing the array to store the Gcode values we extract from the Gcode file
// here we ar using pointers to manage the array because c++ doesn't allow the creation of
// arrays with variable size





//MAIN FUNCTION
int main(int argc, char** argv) {
    // Initialization of ROS
    ros::init(argc, argv, "raven_node");
    ros::NodeHandle nh; // creation of a async spinner for Moveit
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // initiating the move group interface and joint model group
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    goal_pub = nh.advertise<geometry_msgs::Pose>("/goal_pub", 1000);
    print_stat = nh.advertise<std_msgs::String>("/print_stat", 1000);

    //INITIALIZATION OF THE SERIAL PORT AND THE EXTRUDER FOR PRINTING
    if (!initialize_extruder(print_temp)) {
        ROS_INFO_NAMED("error","Could not initialize extruder!");
        return -1;
    }

    // Environment variables (table and objects in environment )
    if (!include_collision_objects_to_env(0.25,0.0,-0.42,1.0,0.90,1.20,0.84,"table")) {
        ROS_INFO_NAMED("error","Could not add objects to moveit!");
        return -2;
    }


    // Gcode reading
    count  = new int;
    *count = Calculate_file_length(path_to_file);

    // Create an array from the Gcode (with all the info)
    const int row = *count-1;
    const int col = 11;
    // Assigning the right pointers for the array which has to carry the Gcode file
    // because now we know the size of the array required
    Original_gcode_array = new float*[row];
    for (int i = 0; i < row; i++)
    {
        Original_gcode_array[i] = new float[col];
    }

    //writing the data from the Gcode into the array created above
    Write_Gcode_to_Array(path_to_file,Original_gcode_array);
    //to send the printing status to the data logging function
    std_msgs::String print_stat_msg;
    print_stat_msg.data= "start";
    print_stat.publish(print_stat_msg);



    //creating the class for the Gcode and setting the universal array with the data
    segment_wise_printer_class obj(PLANNING_GROUP);
    obj.setArray(Original_gcode_array, row, col) ;
    //to finnd the segemnts in the gcode
    std::vector<std::vector<int>> segmentation_array = find_segments(Original_gcode_array);

    //for storing the planned paths and commands for the extruder
    std::vector<trajectory_msgs::JointTrajectory> planned_paths;
    std::vector<std::vector<std::vector<float>>> planned_Gcodes;



    //making the array ofr trajectory planning and the array with commands for extruder
    std::vector<std::vector<float>> Gcode_array_of_segment = obj.init_point();
    //doing the trajectory planning and storing the trajectory
    trajectory_msgs::JointTrajectory planned_path = obj.path_planner();
    //exicuting the plan to move the robot to the home position
    obj.print(planned_path,Gcode_array_of_segment);

    //to get the currnet position of the robot so that it can be added to the start of the trajectory
    //also the origin of the print is claculated at this step so this step cannot be skipped!!
    obj.intialize_for_printing();



    for (const auto& SEGMENT : segmentation_array){
        obj.get_current_joint_position();

        obj.get_start_and_stop(SEGMENT);
        Gcode_array_of_segment = obj.processSegment("straight" , "straight"  );
        planned_path = obj.path_planner();
        planned_paths.push_back(planned_path);
        planned_Gcodes.push_back(Gcode_array_of_segment);

        //obj.update_starting_joint_pose(planned_path);
        obj.print(planned_path,Gcode_array_of_segment);

    }

    for (size_t i = 0; i < planned_paths.size(); ++i){
        if (!planned_paths[i].points.empty()) {
            //obj.print(planned_paths[i],planned_Gcodes[i]);

        }
    }


    sleep(1);
    //send the Gcode to switch off the heater of the extruder
    //set_temperature(0.0);
    ROS_INFO("Done!");
    //to send the printing status to the data logging function
    print_stat_msg.data= "stop";
    print_stat.publish(print_stat_msg);
    //deleting the assignments to the pointer
    for (int i = 0; i < row; i++)
    {
        delete[] Original_gcode_array[i] ;
    }
    delete[] Original_gcode_array;
    return 0;
}



