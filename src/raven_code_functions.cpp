#include "raven_code_functions.h"
#include <iostream>
#include <string>


// The function used to move the robot to the starting point
std::vector<descartes_core::TrajectoryPtPtr> makePath_init(){
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(
            PLANNING_GROUP);
    //code for moving the robot to the initial position using descartes
    EigenSTL::vector_Isometry3d initial_poses;
    const static double time_between_points_initial = 10.55;
    Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();
    init_pose.translation() = Eigen::Vector3d(starting_point[0], starting_point[1], starting_point[2]); //give the desired initial starting pose here!!
    init_pose *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()); // this flips the tool around so that Z is down
    //init_pose *= Eigen::AngleAxisd(8*M_PI/9, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(0*M_PI/9, Eigen::Vector3d::UnitZ());
    initial_poses.push_back(init_pose);
    Eigen::Isometry3d pattern_origin = Eigen::Isometry3d::Identity();
    pattern_origin.translation() = Eigen::Vector3d(0.0,0.0,0.0);

    std::vector<double> joint_group_positions_ss;
    moveit::core::RobotStatePtr current_state_ss = move_group_interface.getCurrentState();
    current_state_ss->copyJointGroupPositions(joint_model_group, joint_group_positions_ss);

    std::vector<descartes_core::TrajectoryPtPtr> result_init;
    std::vector<double> joint_pose = {joint_group_positions_ss[0], joint_group_positions_ss[1], joint_group_positions_ss[2], joint_group_positions_ss[3],joint_group_positions_ss[4], joint_group_positions_ss[5],joint_group_positions_ss[6]};
    result_init.push_back(descartes_core::TrajectoryPtPtr (new descartes_trajectory::JointTrajectoryPt(joint_pose) ));
    for (const auto& init_pose : initial_poses)
    {
        // This creates a trajectory that searches around the tool Z and let's the robot move in that null space
        //descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pattern_origin * pose, time_between_points_initial);
        // This creates a trajectory that is rigid. The tool cannot float and must be at exactly this point.
        descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pattern_origin * init_pose, time_between_points_initial);
        result_init.push_back(pt);
    }
    return result_init;
}

//function used to publish the goals to a topic so that it can be written to a log file
int publishGoal(float Rx,float Ry,float Rz,float TBP,float Px,float Py,float Pz )
{
    geometry_msgs::Pose goal_pose;
    goal_pose.orientation.x = Rx; //rotation about x axis of the extruder
    goal_pose.orientation.y = Ry; //rotation about y axis of the extruder
    goal_pose.orientation.z = Rz; //rotation about z axis of the extruder
    goal_pose.orientation.w = TBP; //time between points parameter calculated inside the function for that segment of motion
    goal_pose.position.x = Px;
    goal_pose.position.y = Py;
    goal_pose.position.z = Pz;
    goal_pub.publish(goal_pose);
    return 0;
}


// function used to calculate the number of points on the gcode file given
// this is important for the proper functioning of this package
// the function return the number of lines on the file, which would then be used to
// calculate the number of points on the file
int Calculate_file_length(std::string path_to_file)
{
    // First thing is to define how many lines are in the Gcode
    std::string line;   // To read each line from code
    int lin_count=0;    // Variable to keep count of each line
    std::ifstream mFile (path_to_file);
    if(mFile.is_open())
    {
        while(mFile.peek()!=EOF)
        {
            getline(mFile, line);
            lin_count++;
        }
        mFile.close();
    }
    return lin_count;
}

// this functions stores the gcode form the file on to an array which would then be used for the printing
int Write_Gcode_to_Array(std::string path_to_file)
{
    std::ifstream file(path_to_file);
    std::string str;
    int lin_num = 0;
    while (std::getline(file, str))
    {

        std::vector <std::string> tokens;
        std::stringstream check1(str);
        std::string intermediate;

        // Tokenizing w.r.t. space ' '
        while(getline(check1, intermediate, ' '))
        {
            tokens.push_back(intermediate);
        }
        for(int i = 0; i < tokens.size(); i++)
        {
            if (lin_num>0)
            {
                float num_float = stof(tokens[i]);
                myArray[lin_num-1][i] = num_float;
            }

        }
        lin_num = lin_num + 1;
    }
    return 0;
}


//INITIALIZATION OF THE SERIAL PORT AND THE EXTRUDER FOR PRINTING
bool initialize_extruder(float print_temp)
    {
    //Opening the connection with the extruder and sending the initial parameters to the extruder
    std::string temp_str;
    std::string temp_str1 = "M109 S" ;
    std::string temp_str2;
    std::string temp_str3 =  "\r\n" ;
    ser.setPort("/dev/ttyACM0");
    ser.setBaudrate(9600);
    serial::Timeout to = serial::Timeout::simpleTimeout(10000);
    ser.setTimeout(to);
    ser.open();
    if(ser.isOpen()){
    ROS_INFO_STREAM("Serial Port initialized");
    }else{
    ROS_INFO_STREAM("Serial Port not working");
    return false;
    }
    temp_str2=std::to_string(print_temp);
    temp_str=temp_str1+temp_str2+temp_str3;
    ser.write(temp_str); //setting the extruder temperature to the printing temperature
    ser.write("G91\r\n"); //setting the extruder to relative positioning
    return true;
    }

//for setting the temperature of the heater
bool set_temperature(float print_temp)
{
    if(ser.isOpen()){
    ROS_INFO_STREAM("Serial Port initialized");
    }else{
    ROS_INFO_STREAM("Serial Port not working");
    return false;
    }
    std::string temp_str;
    std::string temp_str1 = "M109 S" ;
    std::string temp_str2;
    std::string temp_str3 =  "\r\n" ;
    temp_str2=std::to_string(print_temp);
    temp_str=temp_str1+temp_str2+temp_str3;
    ser.write(temp_str); //setting the extruder temperature to the printing temperature
    return true;
} 

//function used to adjustment values which should be added to the positions in the Gcode values to
// reorient the starting point of the gcode to the specified origin point
std::vector<float>   Calculate_origin_adjustment(float move_down_value)
{
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(
            PLANNING_GROUP);
    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose ();

    const static float origin_x = current_pose.pose.position.x;
    const static float origin_y = current_pose.pose.position.y;
    const static float origin_z = current_pose.pose.position.z - move_down_value;  //adjust this ("move_down_value") number if the extruder is too
    // too close or too far from the print bed

    const static float ori_adj_x = origin_x - myArray[0][2] ;
    const static float ori_adj_y = origin_y - myArray[0][3] ;
    const static float ori_adj_z = origin_z - myArray[0][4] ;

    std::vector<float> ori_adj = {ori_adj_x, ori_adj_y, ori_adj_z};
    publishGoal(3.1415926,0.0,0.0,0,current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);

    return ori_adj;
}


//this is the function responsible for sending the gcodes to the extruder control board
//the function takes the number of points in the trajectory as the argument
//the function sends the gcodes stored in the global GcodeArray based on the parameters specified in the array
int sendGcode(int seq_num )
{
    ser.write("G4 S1\r\n");
    for (int j = 0; j <=seq_num+5; j++)
    {

        // We are defining the typical command symbols and string elements of a gcode
        std::string g_str1="G1 F";
        std::string g_strE=" E";
        std::string g_strF;
        std::string g_str2;
        std::string end_str=" \r\n";
        std::string g_str;
        std::string wait_str;
        std::string wait_str1 = "G4 S";
        std::string wait_time_str;


        // We take the gcode values from the gcode array and put it inside strings
        g_strF = std::to_string(GcodeArray[j][0]);
        g_str2=std::to_string(GcodeArray[j][1]);
        g_str=g_str1+g_strF+g_strE+g_str2+end_str;


        wait_time_str = std::to_string(GcodeArray[j][2]);
        wait_str = wait_str1+ wait_time_str+ end_str;


        // This next command actually sends the gcode to the robot
        ser.write(g_str);
        //std::cout<<g_str<<std::endl;
        float delay = 0.0001;
        // Conversion of the time between points in ms
        std::chrono::milliseconds ms{static_cast<long int>((GcodeArray[j][2]- delay )*1000)};
        std::this_thread::sleep_for(ms);
    }
    return 0;
}

// function for creating cartesian trajectory point
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
    using namespace descartes_core;
    using namespace descartes_trajectory;

    return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose), TimingConstraint(dt)) );
}

// function for creating toleranced trajectory point, which would allow the robot to freely orient itself around the tool axis
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose, double dt)
{
    using namespace descartes_core;
    using namespace descartes_trajectory;
    return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI / 12.0, AxialSymmetricPt::Z_AXIS, TimingConstraint(dt)) );
}

//function to execute the trajectory on the robot. The function take the trajectory and the
// path of the follow_joint_trajectory as the arguments
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory, std::string follow_joint_trajectory_action )
{
    // Create a Follow Joint Trajectory Action Client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac (follow_joint_trajectory_action, true);
    if (!ac.waitForServer(ros::Duration(2.0)))
    {
        ROS_ERROR("Could not connect to action server");
        return false;
    }
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    goal.goal_time_tolerance = ros::Duration(1.0);
    return ac.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
}


//the function to attach extruder or other objects to the environment(instead of editing the URDF, not the ideal solution)
// the function takes the position relative to the frame to which object is to be attached of and dimension of the object as;
// the argument along with the name of the object and the frame to which the object is to be attached
bool attach_collision_objects(float Position_x,float Position_y,float Position_z,float Orientation_w,float Dimension_x,float Dimension_y,float Dimension_z,std::string name,std::string Frame )
{
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(
            PLANNING_GROUP);
    moveit_msgs::CollisionObject extruder;
    extruder.id = name;

    shape_msgs::SolidPrimitive extruder_shape;
    extruder_shape.type = extruder_shape.BOX;
    extruder_shape.dimensions.resize(3);
    extruder_shape.dimensions[extruder_shape.BOX_X] = Dimension_x;
    extruder_shape.dimensions[extruder_shape.BOX_Y] = Dimension_y;
    extruder_shape.dimensions[extruder_shape.BOX_Z] = Dimension_z;

    extruder.header.frame_id = move_group_interface.getEndEffectorLink();
    geometry_msgs::Pose extruder_pose;
    extruder_pose.orientation.w = Orientation_w;
    extruder_pose.position.x = Position_x;
    extruder_pose.position.y = Position_y;
    extruder_pose.position.z = Position_z;
    extruder.primitives.push_back(extruder_shape);
    extruder.primitive_poses.push_back(extruder_pose);

    std::string pre = "Attaching the ";
    std::string post =" to the ";
    std::string result = pre + name + post + Frame ;
    ROS_INFO_NAMED("environment", "%s", result.c_str() );
    move_group_interface.attachObject(extruder.id, Frame);
    return true;
}


//the function to add tables or other objects to the environment
// the function takes the position, orientation w and dimension of the object as the argument along with the name of the object
bool include_collision_objects_to_env(float Position_x,float Position_y,float Position_z,float Orientation_w,float Dimension_x,float Dimension_y,float Dimension_z,std::string name )
{
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(
            PLANNING_GROUP);

    moveit_msgs::CollisionObject table;
    table.header.frame_id = move_group_interface.getPlanningFrame();
    table.id = "name";

    shape_msgs::SolidPrimitive table_shape;
    table_shape.type = table_shape.BOX;
    table_shape.dimensions.resize(3);
    table_shape.dimensions[table_shape.BOX_X] = Dimension_x;
    table_shape.dimensions[table_shape.BOX_Y] = Dimension_y;
    table_shape.dimensions[table_shape.BOX_Z] = Dimension_z;

    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = Orientation_w;
    table_pose.position.x = Position_x;
    table_pose.position.y = Position_y;
    table_pose.position.z = Position_z;

    table.primitives.push_back(table_shape);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;

    std::vector <moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(table);
    std::string pre = "Adding the ";
    std::string post =" to the environment";
    std::string result = pre + name + post ;
    ROS_INFO_NAMED("environment", "%s", result.c_str() );
    planning_scene_interface.addCollisionObjects(collision_objects);
    return true;
}
