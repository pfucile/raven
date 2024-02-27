#include "raven_code_functions.h"
#include <iostream>
#include <string>










// class used for prinitng segemnt by segemnt


segment_wise_printer_class::segment_wise_printer_class(const std::string& planning_group_name) :
        move_group_interface(planning_group_name),
        planning_scene_interface(),
        joint_model_group(move_group_interface.getCurrentState()->getJointModelGroup(planning_group_name)),
        myArray(nullptr), // Initialize myarray pointer to nullptr
        rows(0),
        cols(0)
{
    // Constructor logic
}

// Function to set the array
void segment_wise_printer_class::setArray(float** arr, int r, int c) {
    myArray = arr;
    rows = r;
    cols = c;
}

void segment_wise_printer_class::intialize_for_printing() {
    //calculating the value for adjusting the starting point of print to the zero position that we have designated
    std::vector<float>  ori_adj = Calculate_origin_adjustment(move_down_value,myArray);
    ori_adj_x =  ori_adj[0];
    ori_adj_y =  ori_adj[1];
    ori_adj_z =  ori_adj[2];
    moveit::core::RobotStatePtr current_state_ss = move_group_interface.getCurrentState();
    current_state_ss->copyJointGroupPositions(joint_model_group, joint_pose);
    current_pose = move_group_interface.getCurrentPose ();
}
void segment_wise_printer_class::get_current_joint_position() {
    moveit::core::RobotStatePtr current_state_ss = move_group_interface.getCurrentState();
    current_state_ss->copyJointGroupPositions(joint_model_group, joint_pose);
    //gets the current pose
    current_pose = move_group_interface.getCurrentPose ();

    std::cout <<" in the get_current_joint_position() function "<< std::endl;
    std::cout << "Joint values: ";
    for (const auto& name : joint_pose) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    std::cout<<"Position: [" << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << "]" << std::endl;

}
void segment_wise_printer_class::get_start_and_stop(const std::vector<int>& input){
    seq_element_num = 1+input[1]-input[0];
    i = 1+input[1];

}

void segment_wise_printer_class::update_starting_joint_pose(trajectory_msgs::JointTrajectory& joint_solution_to_print){
    std::cout << "Joint values: ";
    for (const auto& name : joint_pose) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    std::cout<<"Position: [" << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << "]" << std::endl;


    // Access the last point in the trajectory
    trajectory_msgs::JointTrajectoryPoint& last_point = joint_solution_to_print.points.back();
    joint_pose = last_point.positions;

    robot_model_loader::RobotModelLoader robot_model_loader(robot_description);
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    // Create a robot state
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
    // Get the joint model group for the end-effector
    const robot_state::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup(PLANNING_GROUP);
    // Set the joint group to joint_pose
    kinematic_state->setJointGroupPositions(joint_model_group, joint_pose);

    // Get the global transform of the end-effector
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform(tcp_frame);
    // Extract translation from the affine transformation
    current_pose.pose.position.x = end_effector_state.translation().x();
    current_pose.pose.position.y = end_effector_state.translation().y();
    current_pose.pose.position.z = end_effector_state.translation().z();
    // Extract rotation from the affine transformation
    Eigen::Quaterniond quat(end_effector_state.rotation());
    current_pose.pose.orientation.x = quat.x();
    current_pose.pose.orientation.y = quat.y();
    current_pose.pose.orientation.z = quat.z();
    current_pose.pose.orientation.w = quat.w();


    std::cout << "Joint values: ";
    for (const auto& name : joint_pose) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    std::cout<<"Position: [" << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << "]" << std::endl;


}


void segment_wise_printer_class::spiral_move_down(float time_to_start){


    std::cout <<" in the spiral_move_down function "<< std::endl;
    std::cout << "Joint values: ";
    for (const auto& name : joint_pose) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    std::cout<<"Position: [" << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << "]" << std::endl;

    pattern_origin = Eigen::Isometry3d::Identity();
    pattern_origin.translation() = Eigen::Vector3d(0.0,0.0,0.0);


    //to add the current pose into the trajectoery
    result.push_back(descartes_core::TrajectoryPtPtr (new descartes_trajectory::JointTrajectoryPt(joint_pose) ));
    pose = Eigen::Isometry3d::Identity();

    GcodeArray.push_back({1000.0f, 1.2f, 0.600f*time_to_start });//preflow
    GcodeArray.push_back({1000.0f, 0.0f, 0.395f*time_to_start});// so that we have time to remove the filament
    GcodeArray.push_back({1000.0f, 0.1f,0.005f*time_to_start}); //so that the first layer sticks to the bed

    float  distance_x = current_pose.pose.position.x - (myArray[i - seq_element_num ][2]+ori_adj_x);
    float  distance_y = current_pose.pose.position.y - (myArray[i - seq_element_num ][3]+ori_adj_y);
    float  distance_z = current_pose.pose.position.z - (myArray[i - seq_element_num ][4]+ori_adj_z);
    float total_distance = sqrt(pow(distance_x,2)+pow(distance_y,2)+pow(distance_z,2));
    int number_of_points =  total_distance*1000*8;
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
        float time_to_point = time_to_start / number_of_points;
        X_pos = current_pose.pose.position.x + (radius * cos(t)) - radius - (((distance_x)/(2*M_PI)) * t);
        Y_pos = current_pose.pose.position.y + (radius * sin(t))- (((distance_y)/(2*M_PI)) * t);
        Z_pos = current_pose.pose.position.z - ((distance_z/(2*M_PI)) * t);
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



void segment_wise_printer_class::direct_move_to_point(float speed){


    std::cout <<" in the direct_move_to_point function "<< std::endl;
    std::cout << "Joint values: ";
    for (const auto& name : joint_pose) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    std::cout<<"Position: [" << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << "]" << std::endl;

    pattern_origin = Eigen::Isometry3d::Identity();
    pattern_origin.translation() = Eigen::Vector3d(0.0,0.0,0.0);
    float  distance_x = current_pose.pose.position.x - (myArray[i - seq_element_num ][2]+ori_adj_x);
    float  distance_y = current_pose.pose.position.y - (myArray[i - seq_element_num ][3]+ori_adj_y);
    float  distance_z = current_pose.pose.position.z - (myArray[i - seq_element_num ][4]+ori_adj_z);
    float total_distance = sqrt(pow(distance_x,2)+pow(distance_y,2)+pow(distance_z,2));
    float time_to_start = total_distance*60.0/speed;

    //to add the current pose into the trajectoery
    result.push_back(descartes_core::TrajectoryPtPtr (new descartes_trajectory::JointTrajectoryPt(joint_pose) ));
    pose = Eigen::Isometry3d::Identity();

    GcodeArray.push_back({1000.0f, 1.2f, 0.600f*time_to_start });//preflow
    GcodeArray.push_back({1000.0f, 0.0f, 0.395f*time_to_start});// so that we have time to remove the filament
    GcodeArray.push_back({1000.0f, 0.1f,0.005f*time_to_start}); //so that the first layer sticks to the bed

    pose.translation() = Eigen::Vector3d(myArray[i - seq_element_num ][2]+ori_adj_x , myArray[i - seq_element_num ][3]+ori_adj_y , myArray[i - seq_element_num ][4]+ori_adj_z );
    pose *= Eigen::AngleAxisd(myArray[i - seq_element_num ][6], Eigen::Vector3d::UnitX()) *Eigen::AngleAxisd(myArray[i - seq_element_num ][7],Eigen::Vector3d::UnitY()); // this flips the tool around so that Z is down
    //descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pattern_origin * pose, time_to_point);
    pt = makeTolerancedCartesianPoint(pattern_origin * pose, time_to_start);
    result.push_back(pt);
    publishGoal(myArray[i - seq_element_num][6], myArray[i - seq_element_num][7],myArray[i - seq_element_num][8], time_to_start, myArray[i - seq_element_num ][2]+ori_adj_x , myArray[i - seq_element_num ][3]+ori_adj_y , myArray[i - seq_element_num ][4]+ori_adj_z);


}

// Function to process input data and create an array
std::vector<std::vector<float>> segment_wise_printer_class::processSegment(std::string starting_style , std::string ending_style  ) {
    result.clear();
    GcodeArray.clear();

    if (starting_style == "spiral"){
        segment_wise_printer_class::spiral_move_down(time_mov_to_start);

    }
    else if (starting_style == "straight"){
        segment_wise_printer_class::direct_move_to_point(1.5);
    }


    for (int j = 1; j <seq_element_num; j++)
    {
        pose = Eigen::Isometry3d::Identity();


        pose.translation() = Eigen::Vector3d(myArray[i-seq_element_num+j][2]+ori_adj_x, myArray[i-seq_element_num+j][3]+ori_adj_y, myArray[i-seq_element_num+j][4]+ori_adj_z);
        // to orient the robot end-effector along the direction as given in the Gcode
        pose *= Eigen::AngleAxisd(myArray[i-seq_element_num+j][6], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(myArray[i-seq_element_num+j][7], Eigen::Vector3d::UnitY());

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
            Extrusion_for_GcodeArray=  extrusion_multiplier*(sqrt(pow(myArray[i-seq_element_num+j][2]-myArray[i-seq_element_num+j-1][2],2)+pow(myArray[i-seq_element_num+j][3]-myArray[i-seq_element_num+j-1][3],2)+pow(myArray[i-seq_element_num+j][4]-myArray[i-seq_element_num+j-1][4],2))*1000)*nozzle_diameter*layer_height*4/(M_PI*(pow(filament_diameter,2))); //to be used with Gcodes that do not have extrusion values.

           feed_rate_for_GcodeArray =  (60 *Extrusion_for_GcodeArray)/ time_between_points ; //to be used with Gcodes that do not have extrusion values.
           time_for_GcodeArray =  time_between_points ;
           GcodeArray.push_back({feed_rate_for_GcodeArray, Extrusion_for_GcodeArray, time_for_GcodeArray });


           }
        else
        {
            feed_rate_for_GcodeArray = 0.0;
            feed_rate_for_GcodeArray =  (60 *Extrusion_for_GcodeArray)/ time_between_points ; //to be used with Gcodes that do not have extrusion values.
            time_for_GcodeArray =  time_between_points ;
            GcodeArray.push_back({feed_rate_for_GcodeArray, Extrusion_for_GcodeArray,time_for_GcodeArray });
        }

        */


        //For Gcodes with Evalue
        time_between_points = (sqrt(pow(myArray[i-seq_element_num+j][2]-myArray[i-seq_element_num+j-1][2],2)+pow(myArray[i-seq_element_num+j][3]-myArray[i-seq_element_num+j-1][3],2)+pow(myArray[i-seq_element_num+j][4]-myArray[i-seq_element_num+j-1][4],2))*1000)/print_velocity ;
        feed_rate_for_GcodeArray =  (60 *(myArray[i-seq_element_num+j][5] - myArray[i-seq_element_num+j-1][5] ))/ time_between_points ; //calculating the material feed rate in mm/minutes for the extruder
        Extrusion_for_GcodeArray =  myArray[i-seq_element_num+j][5] - myArray[i-seq_element_num+j-1][5] ; //for gcodes with Evalue
        time_for_GcodeArray =  time_between_points ;
        GcodeArray.push_back({feed_rate_for_GcodeArray, Extrusion_for_GcodeArray,time_for_GcodeArray });


        // This creates a trajectory that searches around the tool Z and let's the robot move in that null space
        pt = makeTolerancedCartesianPoint(pattern_origin * pose, time_between_points);
        // This creates a trajectory that is rigid. The tool cannot float and must be at exactly this point.
        //descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pattern_origin * pose, time_between_points);
        result.push_back(pt);
        publishGoal(myArray[i-seq_element_num+j][6],myArray[i-seq_element_num+j][7],myArray[i-seq_element_num+j][8],time_between_points,myArray[i-seq_element_num+j][2]+ori_adj_x,myArray[i-seq_element_num+j][3]+ori_adj_y,myArray[i-seq_element_num+j][4]+ori_adj_z);



    }

    pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(myArray[i-1][2]+ori_adj_x,myArray[i-1][3]+ori_adj_y,myArray[i-1][4]+ori_adj_z+.1 );
    pose *= Eigen::AngleAxisd(3.14, Eigen::Vector3d::UnitX()) ;// this flips the tool around so that Z is down
    //pose *= Eigen::AngleAxisd(myArray[i][6], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(myArray[i][7], Eigen::Vector3d::UnitY());
    pt = makeCartesianPoint(pattern_origin * pose, 5.0);
    pt = makeTolerancedCartesianPoint(pattern_origin * pose,time_to_go_back);


    GcodeArray.push_back({4000.0f, -0.5f,0.05f });
    GcodeArray.push_back({4000.0f, -0.1f,0.05f });
    result.push_back(pt);
    publishGoal(myArray[i-1][6],myArray[i-1][7],myArray[i-1][8],time_to_go_back,myArray[i-1][2]+ori_adj_x,myArray[i-1][3]+ori_adj_y,myArray[i-1][4]+ori_adj_z+.01);

    return GcodeArray;



}
std::vector<std::vector<float>> segment_wise_printer_class::init_point(){
    GcodeArray.clear();
    result.clear();


    result = makePath_init();
    return GcodeArray;
}
trajectory_msgs::JointTrajectory segment_wise_printer_class::path_planner() {
    joint_solution.points.clear();
    descartes_core::RobotModelPtr model(new descartes_moveit::IkFastMoveitStateAdapter());
    bool error = false;
    // Initialize the robot model for descartes
    if (!model->initialize(robot_description, PLANNING_GROUP, world_frame, tcp_frame)) {
        ROS_INFO_NAMED("error", "Unable to initialize robot model");
        error = true;
    }

    model->setCheckCollisions(true); // Turns on collision checking.
    //creating the instance of the planner
    planner = new descartes_planner::DensePlanner;

    // Planner initialization
    if (!planner->initialize(model)) {
        ROS_INFO_NAMED("error","Failed to initialize planner");
        error = true;
    }


    //now lets make the plan by passing the trajectory to the planner
    if (!planner->planPath(result))
    {
        ROS_ERROR("Could not solve for a valid path");
        error = true;
    }

    // now we have to extract the calculated path
    if (!planner->getPath(plan_result))
    {
        ROS_ERROR("Could not retrieve path");
        error = true;

    }

    // 5. Translate the result into something that you can execute. In ROS land, this means that we turn the result into
    // a trajectory_msgs::JointTrajectory
    std::vector <std::string> names;
    names = joint_model_group->getVariableNames();

    // Create a JointTrajectory
    joint_solution.joint_names = names;

    // Define a default velocity. Descartes points without specified timing will use this value to limit the
    // fastest moving joint. This usually effects the first point in your path the most.
    if (!descartes_utilities::toRosJointPoints(*model, plan_result, default_joint_vel, joint_solution.points))
    {
        ROS_ERROR("Unable to convert Descartes trajectory to joint points");
        error = true;
    }
    //to send the printing status to the data logging function
    std_msgs::String print_stat_msg;
    print_stat_msg.data= "trajectory calculation done";
    print_stat.publish(print_stat_msg);
    std::cout<< "trajectory calculation done"<<std::endl;
    result.clear();
    GcodeArray.clear();
    if (!error){
        return joint_solution;
    }else{
        joint_solution.points.clear();
        return joint_solution;
    }
}
void segment_wise_printer_class::print(trajectory_msgs::JointTrajectory joint_solution_to_print ,std::vector<std::vector<float>> Gcode_array_of_segment){
    auto f = std::async(std::launch::async, sendGcode , std::ref(Gcode_array_of_segment));
    if (!executeTrajectory(joint_solution_to_print , follow_joint_trajectory_action)) {
        ROS_ERROR("Could not execute trajectory!");
    }
    std::cout<< "printing of segment done"<<std::endl;
}



















// Definition of rellavant function



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

    std::vector<double> joint_pose;
    std::vector<descartes_core::TrajectoryPtPtr> result_init;
    moveit::core::RobotStatePtr current_state_ss = move_group_interface.getCurrentState();
    current_state_ss->copyJointGroupPositions(joint_model_group, joint_pose);
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
int Write_Gcode_to_Array(std::string path_to_file,float** myArray)
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
std::vector<float>   Calculate_origin_adjustment(float move_down_value, float** myArray)
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
int sendGcode(std::vector<std::vector<float>>& GcodeArray_to_print )
{
    ser.write("G4 S1\r\n");
    for (int j = 0; j < GcodeArray_to_print.size(); j++)
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
        g_strF = std::to_string(GcodeArray_to_print[j][0]);
        g_str2=std::to_string(GcodeArray_to_print[j][1]);
        g_str=g_str1+g_strF+g_strE+g_str2+end_str;


        wait_time_str = std::to_string(GcodeArray_to_print[j][2]);
        wait_str = wait_str1+ wait_time_str+ end_str;


        // This next command actually sends the gcode to the robot
        ser.write(g_str);
        //std::cout<<g_str<<std::endl;
        float delay = 0.0001;
        // Conversion of the time between points in ms
        std::chrono::milliseconds ms{static_cast<long int>((GcodeArray_to_print[j][2]- delay )*1000)};
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




//the function used to identify the segemnts and save the starting point and the endpoint in an array

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