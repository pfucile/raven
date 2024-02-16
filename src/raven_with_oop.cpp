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
float print_temp = 195; //printing temperature
float nozzle_diameter = 0.4; //mm
float layer_height = 0.3; //mm
float extrusion_multiplier = 0.6; //mm
float filament_diameter = 1.75; //mm
float starting_point[] = {0.35, 0.25, 0.15}; //give the desired initial starting pose here!!
float move_down_value = 0.0825;  //adjust this number if the extruder is too close or too far from the print bed
float time_mov_to_start = 10 ; //time taken to move from the waiting point to the first point on the segment
float time_to_go_back = 5.0;   //time taken to move away after print
EigenSTL::vector_Isometry3d pattern_poses;
double time_between_points = 0.15;

int* count;
const double default_joint_vel = 0.05; // Define the default speed in rad/s. We define this if the time between points is not provide
float ori_adj_x ;
float ori_adj_y ;
float ori_adj_z ;

// Create an object from the Serial class for the extruder
serial::Serial ser;

// Create objects and functions for the trajectory planning
std::vector<descartes_core::TrajectoryPtPtr> makePath();
std::vector<descartes_core::TrajectoryPtPtr> makePath_init();
descartes_planner::DensePlanner* planner;
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Isometry3d& pose, double dt);
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt);
descartes_core::RobotModelPtr model(new descartes_moveit::IkFastMoveitStateAdapter());
ros::Publisher goal_pub;
ros::Publisher print_stat;



//Initialization of different functions
std::vector<float>  Calculate_origin_adjustment(float move_down_value , float** myarray); //Initialization of the function to calculate the correction value to set the origin of the print to the designated point
int Write_Gcode_to_Array(std::string path_to_file,float** myarray);//Initialization of the function to load the gcode to a array for the functions to use
bool execute_gcode_sequence_by_sequence(float ori_adj_x, float ori_adj_y, float ori_adj_z); //Initialization of the function to execute the actual Gcode
; //Initialization of the function to inlude the extruder nad table to environment
bool attach_collision_objects(float Position_x,float Position_y,float Position_z,float Orientation_w,float Dimension_x,float Dimension_y,float Dimension_z,std::string name,std::string Frame );
bool include_collision_objects_to_env(float Position_x,float Position_y,float Position_z,float Orientation_w,float Dimension_x,float Dimension_y,float Dimension_z,std::string name );
int Calculate_file_length(std::string path_to_file); // Initialization of the function to calculate the length of the Gcode
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory, std::string follow_joint_trajectory_action ); // upload of the trajectory to be executed - Sends a ROS trajectory to the robot controller
int publishGoal(float Rx,float Ry,float Rz,float TBP,float Px,float Py,float Pz );
bool initialize_extruder(float print_temp);

//the gcode file
std::string path_to_file = "../xarm_ws/src/raven/scripts/Converted_file.txt";

std::string follow_joint_trajectory_action = "/xarm/xarm7_traj_controller/follow_joint_trajectory";

// Defining again right arm group, because move_group_interface from
// MoveIt and Descartes need different initialization types
// definition of the robot variables.
const std::string robot_description = "robot_description";
const std::string PLANNING_GROUP = "xarm7"; // Attribution of planning group variable
const std::string world_frame = "world";
const std::string tcp_frame = "extruder_tip";

std::vector <std::string> names;
std::vector <double> seed;
// Initialization of the Gcode objects and functions
int sendGcode(int seq_num  );
float GcodeArray[10000][3]; // array to store the exact values used for controlling the extruder
// the fists column is the feedrate second is the length of filament to be extruded for that segment
// the third on is the time required for the motion in that segment

float** Original_gcode_array; // Initializing the array to store the Gcode values we extract from the Gcode file
// here we ar using pointers to manage the array because c++ doesn't allow the creation of
// arrays with variable size

std::vector<std::vector<int>> find_segments(float** myArray); // Initialization of the function to find the relevant segments







class segment_wise_printer_class {
private:
    const std::string PLANNING_GROUP = "xarm7";
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group;
    float GcodeArray[10000][3];
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

public:
    // Constructor
    segment_wise_printer_class(const std::string& planning_group_name) :
            move_group_interface(planning_group_name),
            planning_scene_interface(), // Optionally, initialize any other members here
            joint_model_group(move_group_interface.getCurrentState()->getJointModelGroup(planning_group_name)),
            myArray(nullptr), // Initialize myarray pointer to nullptr
            rows(0),
            cols(0)
    {
        // Constructor logic
    }

    // Function to set the array
    void setArray(float** arr, int r, int c) {
        myArray = arr;
        rows = r;
        cols = c;
    }

    void printArray() {
        std::cout << "Printing array:" << std::endl;
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                std::cout << myArray[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }
    void get_current_joint_position() {
        moveit::core::RobotStatePtr current_state_ss = move_group_interface.getCurrentState();
        current_state_ss->copyJointGroupPositions(joint_model_group, joint_group_positions_ss);
        std::vector<double> joint_pose = {joint_group_positions_ss[0], joint_group_positions_ss[1], joint_group_positions_ss[2], joint_group_positions_ss[3],joint_group_positions_ss[4], joint_group_positions_ss[5],joint_group_positions_ss[6]};
        current_pose = move_group_interface.getCurrentPose ();
    }
    void get_start_and_stop(const std::vector<int>& input){
        seq_element_num = 1+input[1]-input[0];
        i = 1+input[1];
    }
    void spiral_move_down(){
        Eigen::Isometry3d pattern_origin = Eigen::Isometry3d::Identity();
        pattern_origin.translation() = Eigen::Vector3d(0.0,0.0,0.0);
        result.push_back(descartes_core::TrajectoryPtPtr (new descartes_trajectory::JointTrajectoryPt(joint_pose) ));
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

        GcodeArray[0][0] = 1000 ;
        GcodeArray[0][1] = 0.7 ; //preflow
        GcodeArray[0][2] = 0.6*time_mov_to_start ;
        GcodeArray[1][0] = 1000 ;
        GcodeArray[1][1] = 0 ;
        GcodeArray[1][2] = 0.395*time_mov_to_start;  // so that we have time to remove the filament
        GcodeArray[2][0] = 1000 ;
        GcodeArray[2][1] = 0.5 ; //so that the first layer sticks to the bed
        GcodeArray[2][2] = 0.005*time_mov_to_start ;

        float  distance_x = current_pose.pose.position.x - (myArray[i - seq_element_num ][2]+ori_adj_x);
        float  distance_y = current_pose.pose.position.y - (myArray[i - seq_element_num ][3]+ori_adj_y);
        float  distance_z = current_pose.pose.position.z - (myArray[i - seq_element_num ][4]+ori_adj_z);
        int number_of_points =   distance_z*1000*8;
        float t = 0;
        float Rot_x = M_PI;
        float Rot_y = 0;
        float delta_Rot_x = (myArray[i - seq_element_num ][6] - Rot_x)/number_of_points ;
        float delta_Rot_y = (myArray[i - seq_element_num ][7] - Rot_y)/number_of_points ;
        float radius = 0.05;
        float X_pos = 0.0;
        float Y_pos = 0.0;
        float Z_pos = 0.0;

        for (int k=0; k<number_of_points;k++)
        {
            pose = Eigen::Isometry3d::Identity();
            float time_to_point = time_mov_to_start / number_of_points;
            float X_pos = current_pose.pose.position.x + (radius * cos(t)) - radius - (((distance_x)/(2*M_PI)) * t);
            float Y_pos = current_pose.pose.position.y + (radius * sin(t))- (((distance_y)/(2*M_PI)) * t);
            float Z_pos = current_pose.pose.position.z - ((distance_z/(2*M_PI)) * t);
            Rot_x = Rot_x + delta_Rot_x;
            Rot_y = Rot_y + delta_Rot_y;
            t = t + (2 * M_PI / number_of_points);
            pose.translation() = Eigen::Vector3d(X_pos , Y_pos , Z_pos );
            pose *= Eigen::AngleAxisd(Rot_x, Eigen::Vector3d::UnitX()) *Eigen::AngleAxisd(Rot_y,Eigen::Vector3d::UnitY()); // this flips the tool around so that Z is down
            //descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pattern_origin * pose, time_to_point);
            pt = makeTolerancedCartesianPoint(pattern_origin * pose, time_to_point);
            result.push_back(pt);
            publishGoal(myArray[i - seq_element_num][6], myArray[i - seq_element_num][7],myArray[i - seq_element_num][8], time_to_point, X_pos , Y_pos ,Z_pos);

        }


    }
    // Function to process input data and create an array
    void processData() {


        for (int j = 1; j <seq_element_num; j++)
        {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

            pose.translation() = Eigen::Vector3d(myArray[i-seq_element_num+j][2]+ori_adj_x, myArray[i-seq_element_num+j][3]+ori_adj_y, myArray[i-seq_element_num+j][4]+ori_adj_z);
            ROS_INFO(" error debugging ! - 1");
            // to orient the robot end-effector along the direction as given in the Gcode
            pose *= Eigen::AngleAxisd(myArray[i-seq_element_num+j][6], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(myArray[i-seq_element_num+j][7], Eigen::Vector3d::UnitY());

            ROS_INFO(" error debugging ! -2 ");
            if (myArray[i-seq_element_num+j][1] <= max_print_speed)
            {
                print_velocity = (myArray[i-seq_element_num+j][1]*1000)/60; // converting from m/min to mm/s

            }
            else
            {
                print_velocity = (max_print_speed*1000)/60;// converting from m/min to mm/s
            }


            /*

            //to be used with Gcodes that do not have extrusion values.
            time_between_points = (sqrt(pow(myArray[i-seq_element_num+j][2]-myArray[i-seq_element_num+j-1][2],2)+pow(myArray[i-seq_element_num+j][3]-myArray[i-seq_element_num+j-1][3],2)+pow(myArray[i-seq_element_num+j][4]-myArray[i-seq_element_num+j-1][4],2))*1000)/print_velocity ;
            //to be used with Gcodes that do not have extrusion values.
            E = extrusion_multiplier * sqrt(pow(myArray[i][2]-myArray[i-1][2],2)+pow(myArray[i][3]-myArray[i-1][3],2)+pow(myArray[i][4]-myArray[i-1][4],2))*nozzle_diameter*layer_height*4/(M_PI*(pow(filament_diameter,2))); //to be used with Gcodes that do not have extrusion values.
            if (myArray[i-seq_element_num+j][0] == 1)
            {
                GcodeArray[j+3][1]=  extrusion_multiplier*(sqrt(pow(myArray[i-seq_element_num+j][2]-myArray[i-seq_element_num+j-1][2],2)+pow(myArray[i-seq_element_num+j][3]-myArray[i-seq_element_num+j-1][3],2)+pow(myArray[i-seq_element_num+j][4]-myArray[i-seq_element_num+j-1][4],2))*1000)*nozzle_diameter*layer_height*4/(M_PI*(pow(filament_diameter,2))); //to be used with Gcodes that do not have extrusion values.
            }
            else
            {
                GcodeArray[j+3][1] = 0.0;
            }
            GcodeArray[j+3][0] =  (60 *GcodeArray[j+3][1])/ time_between_points ; //to be used with Gcodes that do not have extrusion values.
            GcodeArray[j+3][2] =  time_between_points ;
            */


            //For Gcodes with Evalue
            ROS_INFO(" error debugging ! -3 ");
            time_between_points = (sqrt(pow(myArray[i-seq_element_num+j][2]-myArray[i-seq_element_num+j-1][2],2)+pow(myArray[i-seq_element_num+j][3]-myArray[i-seq_element_num+j-1][3],2)+pow(myArray[i-seq_element_num+j][4]-myArray[i-seq_element_num+j-1][4],2))*1000)/print_velocity ;
            ROS_INFO(" error debugging ! -4 ");
            GcodeArray[j+3][0] =  (60 *(myArray[i-seq_element_num+j][5] - myArray[i-seq_element_num+j-1][5] ))/ time_between_points ; //calculating the material feed rate in mm/minutes for the extruder
            ROS_INFO(" error debugging ! -5 ");
            GcodeArray[j+3][1] =  myArray[i-seq_element_num+j][5] - myArray[i-seq_element_num+j-1][5] ; //for gcodes with Evalue
            ROS_INFO(" error debugging ! -6 ");
            GcodeArray[j+3][2] =  time_between_points ;
            ROS_INFO(" error debugging ! -7 ");


            // This creates a trajectory that searches around the tool Z and let's the robot move in that null space
            descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pattern_origin * pose, time_between_points);
            // This creates a trajectory that is rigid. The tool cannot float and must be at exactly this point.
            //descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pattern_origin * pose, time_between_points);
            result.push_back(pt);
            publishGoal(myArray[i-seq_element_num+j][6],myArray[i-seq_element_num+j][7],myArray[i-seq_element_num+j][8],time_between_points,myArray[i-seq_element_num+j][2]+ori_adj_x,myArray[i-seq_element_num+j][3]+ori_adj_y,myArray[i-seq_element_num+j][4]+ori_adj_z);



        }

        pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(myArray[i-1][2]+ori_adj_x,myArray[i-1][3]+ori_adj_y,myArray[i-1][4]+ori_adj_z+.1 );
        pose *= Eigen::AngleAxisd(3.14, Eigen::Vector3d::UnitX()) ;// this flips the tool around so that Z is down
        //pose *= Eigen::AngleAxisd(myArray[i][6], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(myArray[i][7], Eigen::Vector3d::UnitY());
        //pt = makeCartesianPoint(pattern_origin * pose, 5.0);
        pt = makeTolerancedCartesianPoint(pattern_origin * pose,time_to_go_back);

        GcodeArray[seq_element_num+3][0] = 4000 ;
        GcodeArray[seq_element_num+3][1] = -0.5 ;
        GcodeArray[seq_element_num+3][2] = 0.05 ;
        GcodeArray[seq_element_num+4][0] = 4000 ;
        GcodeArray[seq_element_num+4][1] = -0.1 ;
        GcodeArray[seq_element_num+4][2] = 0.5 ;
        result.push_back(pt);
        publishGoal(myArray[i-1][6],myArray[i-1][7],myArray[i-1][8],time_to_go_back,myArray[i-1][2]+ori_adj_x,myArray[i-1][3]+ori_adj_y,myArray[i-1][4]+ori_adj_z+.05);

    }

    void path_planner() {
        //now lets make the plan by passing the trajectory to the planner
        if (!planner->planPath(result))
        {
            ROS_ERROR("Could not solve for a valid path");
        }
        // now we have to extract the calculated path
        std::vector<descartes_core::TrajectoryPtPtr> plan_result;
        if (!planner->getPath(plan_result))
        {
            ROS_ERROR("Could not retrieve path");
        }
        // 5. Translate the result into something that you can execute. In ROS land, this means that we turn the result into
        // a trajectory_msgs::JointTrajectory
        std::vector <std::string> names;
        names = joint_model_group->getVariableNames();

        // Create a JointTrajectory
        trajectory_msgs::JointTrajectory joint_solution;
        joint_solution.joint_names = names;
        // Define a default velocity. Descartes points without specified timing will use this value to limit the
        // fastest moving joint. This usually effects the first point in your path the most.
        if (!descartes_utilities::toRosJointPoints(*model, plan_result, default_joint_vel, joint_solution.points))
        {
            ROS_ERROR("Unable to convert Descartes trajectory to joint points");
        }
        //to send the printing status to the data logging function
        std_msgs::String print_stat_msg;
        print_stat_msg.data= "trajectory calculation done";
        print_stat.publish(print_stat_msg);
        std::cout<< "trajectory calculation done"<<std::endl;
    }
    void print(){
        auto f = std::async(std::launch::async, sendGcode ,seq_element_num );
        if (!executeTrajectory(joint_solution , follow_joint_trajectory_action)) {
            ROS_ERROR("Could not execute trajectory!");
        }
    }
};




//MAIN FUNCTION
int main(int argc, char** argv) {
    // Initialization of ROS
    ros::init(argc, argv, "raven_node");
    ros::NodeHandle nh; // creation of a async spinner for Moveit
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Creation of the kinematic model for the robot. we are using IKFast as solver
    descartes_core::RobotModelPtr model(new descartes_moveit::IkFastMoveitStateAdapter());
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
    // Environment variables (extruder block)
    //if (!attach_collision_objects(0.037,0.0,0.073,1.0,0.154,0.080,0.107,"extruder","right_hand")) {
    //    ROS_INFO_NAMED("error","Could not add objects to moveit!");
    //    return -3;
    //}

    // Initialize the robot model for descartes
    if (!model->initialize(robot_description, PLANNING_GROUP, world_frame, tcp_frame)) {
        ROS_INFO_NAMED("error", "Unable to initialize robot model");
        return -4;
    }

    model->setCheckCollisions(true); // Turns on collision checking.
    // using the function makePath_init() to create the path to the waiting point
    std::vector <descartes_core::TrajectoryPtPtr> init_points = makePath_init();
    //creating the instance of the planner
    planner = new descartes_planner::DensePlanner;

    // Planner initialization
    if (!planner->initialize(model)) {
        ROS_INFO_NAMED("error","Failed to initialize planner");
        return -5;
    }
    // planPath(). This function takes your input trajectory and expands it into
    // a large kinematic "graph". Failures at this point indicate that the
    // input path may not have solutions at a given point (because of reach/collision)
    // or has two points with no way to connect them.

    if (!planner->planPath(init_points)) {
        ROS_INFO_NAMED("error","Could not solve for a valid path");
        return -6;
    }

    // getPath looks for the minimum cost (best) trajectory resulting
    // from the planPath of the previous step.

    std::vector <descartes_core::TrajectoryPtPtr> init_result;
    if (!planner->getPath(init_result)) {
        ROS_INFO_NAMED("error","Could not retrieve path");
        return -7;
    }
    // Translate the trajectory into an executable one for ROS interfaces. We are using
    // standard ROS ones --> trajectory_msgs and control_msgs for the actual execution

    // First, we retrieve the joint names
    std::vector <std::string> names;
    names = joint_model_group->getVariableNames();

    trajectory_msgs::JointTrajectory init_joint_solution;
    // here we are putting the joint names in the init_joint_solution class variable
    init_joint_solution.joint_names = names;

    if (!descartes_utilities::toRosJointPoints(*model, init_result, default_joint_vel, init_joint_solution.points)) {
        ROS_INFO_NAMED("error","Unable to convert Descartes trajectory to joint points");
        return -8;
    }

    // We send the trajectory to ROS
    if (!executeTrajectory(init_joint_solution,follow_joint_trajectory_action)) {
        ROS_INFO_NAMED("error","Could not execute trajectory!");
        return -9;
    }
    sleep(2.0);

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
    //calculating the value for adjusting the starting point of print to the zero position that we have designated
    std::vector<float>  ori_adj = Calculate_origin_adjustment(move_down_value,Original_gcode_array);
    float ori_adj_x =  ori_adj[0];
    float ori_adj_y =  ori_adj[1];
    float ori_adj_z =  ori_adj[2];




    //creating the class for the Gcode
    segment_wise_printer_class obj(PLANNING_GROUP);
    obj.setArray(Original_gcode_array, row, col) ;


    std::vector<std::vector<int>> segmentation_array = find_segments(Original_gcode_array);

    for (const auto& SEGMENT : segmentation_array){
        obj.get_current_joint_position();
        obj.get_start_and_stop(SEGMENT);
        obj.spiral_move_down();
        obj.processData();
        obj.path_planner();

        obj.print();

    }


    sleep(1);
    //send the Gcode to switch off the heater of the extruder
    set_temperature(0.0);
    ROS_INFO("Done!");
    //to send the printing status to the data logging function
    print_stat_msg.data= "stop";
    print_stat.publish(print_stat_msg);
    //deleting the assignments to a pointer
    for (int i = 0; i < row; i++)
    {
        delete[] Original_gcode_array[i] ;
    }
    delete[] Original_gcode_array;
    return 0;
}


std::vector<std::vector<int>> find_segments(float** myArray) {
    int seq_element_num = 0;
    float E_subtotal = 0.0;
    float E_previous = 0.0;
    float E = 0.0;
    float E_str_float = 0.0;
    float prev_seg_num = myArray[0][9];
    std::vector <std::vector<int>> segmentation_array; //to store the start point and the end point of the segment
    for (int i = 0; i < *count-1; i++) {
        if (myArray[i][9] == prev_seg_num and seq_element_num <= 10000 and i < (*count-2)) {
            seq_element_num == seq_element_num++;
            E = myArray[i][5] - E_previous;
            E_subtotal = E_subtotal + E;
            E_previous = myArray[i][5];
            prev_seg_num = myArray[i][9];
        }
        else if (myArray[i][9] != prev_seg_num or seq_element_num > 10000 or i == (*count-2)) {
            std::vector<int> newRow;
            newRow.push_back(i - seq_element_num);
            newRow.push_back(i - 1);
            segmentation_array.push_back(newRow);
            seq_element_num = 1;
            E_subtotal = 0;
            E = myArray[i][5] - E_previous;
            E_subtotal = E_subtotal + E;
            E_previous = myArray[i][5];
            prev_seg_num = myArray[i][9];
        }

    }
    return segmentation_array;
}
