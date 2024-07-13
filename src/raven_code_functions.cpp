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
        
    }
    void segment_wise_printer_class::set_start_and_stop(const std::vector<int>& input){
        seq_element_num = 1+input[1]-input[0];
        i = 1+input[1];
 
    }
    
    void segment_wise_printer_class::update_starting_joint_pose(trajectory_msgs::JointTrajectory& joint_solution_to_print){

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


    }
    
    std::vector<double> segment_wise_printer_class::find_pose_forward_kinematics(std::vector<double> joint_pose){
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
        // Extract rotation (quaternion) from the affine transformation
	Eigen::Quaterniond quaternion(end_effector_state.rotation());
	// Convert quaternion to RPY angles
	double roll, pitch, yaw;
	tf2::Matrix3x3(tf2::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w())).getRPY(roll, pitch, yaw);

        // Extract translation from the affine transformation
        std::cout<<end_effector_state.translation().x()<<end_effector_state.translation().y()<<end_effector_state.translation().z()<<std::endl;
        std::vector<double> pose_calculated = {end_effector_state.translation().x(),end_effector_state.translation().y(),end_effector_state.translation().z(),roll, pitch, yaw};
        return pose_calculated;
    }

    
    void segment_wise_printer_class::spiral_move_down(float time_to_start){
        
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
        int number_of_points =  total_distance*1000;
        std::cout<<" number_of_points  : " << number_of_points<<std::endl;
        float t = 0;
        float Rot_x = myArray[i - seq_element_num ][6];
        float Rot_y = myArray[i - seq_element_num ][7];
        float delta_Rot_x = (myArray[i - seq_element_num ][6] - Rot_x)/number_of_points ;
        float delta_Rot_y = (myArray[i - seq_element_num ][7] - Rot_y)/number_of_points ;
        float radius = 0.02;
        float X_pos = 0.0;
        float Y_pos = 0.0;
        float Z_pos = 0.0;
        float time_to_point = time_to_start / number_of_points;

        for (int k=0; k<number_of_points;k++)
        {
            pose = Eigen::Isometry3d::Identity();
            X_pos = current_pose.pose.position.x + (radius * cos(t)) - radius - (((distance_x)/(2*M_PI)) * t);
            Y_pos = current_pose.pose.position.y + (radius * sin(t))- (((distance_y)/(2*M_PI)) * t);
            Z_pos = current_pose.pose.position.z - ((distance_z/(2*M_PI)) * t);
            
            Rot_x = Rot_x + delta_Rot_x;
            Rot_y = Rot_y + delta_Rot_y;
            
            std::cout<<" delta_Rot_x  : " << delta_Rot_x << "  delta_Rot_y  :  " <<delta_Rot_y<<std::endl;
            t = t + (2 * M_PI / number_of_points);
            pose.translation() = Eigen::Vector3d(X_pos , Y_pos , Z_pos );
            pose *= Eigen::AngleAxisd(Rot_x, Eigen::Vector3d::UnitX()) *Eigen::AngleAxisd(Rot_y,Eigen::Vector3d::UnitY()); // this flips the tool around so that Z is down
            //pt = makeCartesianPoint(pattern_origin * pose, time_to_point);
            pt = makeTolerancedCartesianPoint(pattern_origin * pose, time_to_point);
            result.push_back(pt);
            publishGoal(myArray[i - seq_element_num][6], myArray[i - seq_element_num][7],myArray[i - seq_element_num][8], time_to_point, X_pos , Y_pos ,Z_pos);
            
            }


    }


    void segment_wise_printer_class::direct_move_to_point(float time_to_start){
        
        pattern_origin = Eigen::Isometry3d::Identity();
        pattern_origin.translation() = Eigen::Vector3d(0.0,0.0,0.0);
        
        
        //to add the current pose into the trajectoery
        result.push_back(descartes_core::TrajectoryPtPtr (new descartes_trajectory::JointTrajectoryPt(joint_pose) ));
        pose = Eigen::Isometry3d::Identity();

        GcodeArray.push_back({calculate_F_for_E_and_time(0.400f*time_to_start,0.0f), 0.0f, 0.400f*time_to_start });//preflow
        GcodeArray.push_back({calculate_F_for_E_and_time(0.599f*time_to_start,0.0f), 0.0f, 0.599f*time_to_start});// so that we have time to remove the filament
        GcodeArray.push_back({calculate_F_for_E_and_time(0.001f*time_to_start,4.5f), 4.5f,0.001f*time_to_start}); //so that the first layer sticks to the bed
	
        float  distance_x = current_pose.pose.position.x - (myArray[i - seq_element_num ][2]+ori_adj_x);
        float  distance_y = current_pose.pose.position.y - (myArray[i - seq_element_num ][3]+ori_adj_y);
        float  distance_z = current_pose.pose.position.z - (myArray[i - seq_element_num ][4]+ori_adj_z);
        float total_distance = sqrt(pow(distance_x,2)+pow(distance_y,2)+pow(distance_z,2));
        int number_of_points =  200;
        std::cout<<" number_of_points  : " << number_of_points<<std::endl;
        float t = 0;
        //float Rot_x = M_PI;
        //float Rot_y = 0.0;
        float Rot_x = myArray[i - seq_element_num ][6];
        float Rot_y = myArray[i - seq_element_num ][7];
        float delta_Rot_x = (myArray[i - seq_element_num ][6] - Rot_x)/number_of_points ;
        float delta_Rot_y = (myArray[i - seq_element_num ][7] - Rot_y)/number_of_points ;
        float radius = 0.0;
        float X_pos = 0.0;
        float Y_pos = 0.0;
        float Z_pos = 0.0;
        float time_to_point = time_to_start / number_of_points;

        for (int k=0; k<number_of_points;k++)
        {
            pose = Eigen::Isometry3d::Identity();
            X_pos = current_pose.pose.position.x + (radius * cos(t)) - radius - (((distance_x)/(2*M_PI)) * t);
            Y_pos = current_pose.pose.position.y + (radius * sin(t))- (((distance_y)/(2*M_PI)) * t);
            Z_pos = current_pose.pose.position.z - ((distance_z/(2*M_PI)) * t);
            
            Rot_x = Rot_x + delta_Rot_x;
            Rot_y = Rot_y + delta_Rot_y;
            
            std::cout<<" delta_Rot_x  : " << delta_Rot_x << "  delta_Rot_y  :  " <<delta_Rot_y<<std::endl;
            t = t + (2 * M_PI / number_of_points);
            pose.translation() = Eigen::Vector3d(X_pos , Y_pos , Z_pos );
            pose *= Eigen::AngleAxisd(Rot_x, Eigen::Vector3d::UnitX()) *Eigen::AngleAxisd(Rot_y,Eigen::Vector3d::UnitY()); // this flips the tool around so that Z is down
            //pt = makeCartesianPoint(pattern_origin * pose, time_to_point);
            pt = makeTolerancedCartesianPoint(pattern_origin * pose, time_to_point);
            result.push_back(pt);
            publishGoal(myArray[i - seq_element_num][6], myArray[i - seq_element_num][7],myArray[i - seq_element_num][8], time_to_point, X_pos , Y_pos ,Z_pos);

            }


    }



    // Function to process input data and create an array
    std::vector<std::vector<float>> segment_wise_printer_class::processSegment(std::string starting_style , std::string ending_style , float retract_distance ) {
        result.clear();
        GcodeArray.clear(); 

        if (starting_style == "spiral"){
            segment_wise_printer_class::spiral_move_down(time_mov_to_start);
            
        }
        else if (starting_style == "straight"){
            segment_wise_printer_class::direct_move_to_point(time_mov_to_start);
        }
        
            
        for (int j = 1; j <seq_element_num; j++)
        {
            pose = Eigen::Isometry3d::Identity();
            
            
            pose.translation() = Eigen::Vector3d(myArray[i-seq_element_num+j][2]+ori_adj_x, myArray[i-seq_element_num+j][3]+ori_adj_y, myArray[i-seq_element_num+j][4]+ori_adj_z);
            // to orient the robot end-effector along the direction as given in the Gcode
            pose *= Eigen::AngleAxisd(myArray[i-seq_element_num+j][6], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(myArray[i-seq_element_num+j][7], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(myArray[i-seq_element_num+j][8], Eigen::Vector3d::UnitZ());
            
            if (myArray[i-seq_element_num+j][1] <= max_print_speed)
            {
                print_velocity = (myArray[i-seq_element_num+j][1]*1000.0f)/60.0f; // converting from m/min to mm/s

            }
            else
            {
                print_velocity = (max_print_speed*1000.0f)/60.0f;// converting from m/min to mm/s
            }


            /*
             
            //to be used with Gcodes that do not have extrusion values.
            time_between_points = (sqrt(pow(myArray[i-seq_element_num+j][2]-myArray[i-seq_element_num+j-1][2],2)+pow(myArray[i-seq_element_num+j][3]-myArray[i-seq_element_num+j-1][3],2)+pow(myArray[i-seq_element_num+j][4]-myArray[i-seq_element_num+j-1][4],2))*1000)/print_velocity ;
            //to be used with Gcodes that do not have extrusion values.
            //E = extrusion_multiplier * sqrt(pow(myArray[i][2]-myArray[i-1][2],2)+pow(myArray[i][3]-myArray[i-1][3],2)+pow(myArray[i][4]-myArray[i-1][4],2))*nozzle_diameter*layer_height*4/(M_PI*(pow(filament_diameter,2))); 
            if (myArray[i-seq_element_num+j][0] == 1)
            {
                Extrusion_for_GcodeArray=  1.0*(sqrt(pow(myArray[i-seq_element_num+j][2]-myArray[i-seq_element_num+j-1][2],2)+pow(myArray[i-seq_element_num+j][3]-myArray[i-seq_element_num+j-1][3],2)+pow(myArray[i-seq_element_num+j][4]-myArray[i-seq_element_num+j-1][4],2))*1000)*nozzle_diameter*layer_height*4/(M_PI*(pow(filament_diameter,2))); 
               
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
            time_between_points = (sqrt(pow(myArray[i-seq_element_num+j][2]-myArray[i-seq_element_num+j-1][2],2)+pow(myArray[i-seq_element_num+j][3]-myArray[i-seq_element_num+j-1][3],2)+pow(myArray[i-seq_element_num+j][4]-myArray[i-seq_element_num+j-1][4],2))*1000.0f)/print_velocity ;
            
            Extrusion_for_GcodeArray =extrusion_multiplier*(  myArray[i-seq_element_num+j][5] - myArray[i-seq_element_num+j-1][5]); 
            feed_rate_for_GcodeArray =  (60.0f *Extrusion_for_GcodeArray)/ time_between_points ; //calculating the material feed rate in mm/minutes for the extruder
            //for gcodes with Evalue
            time_for_GcodeArray =  time_between_points ;
            GcodeArray.push_back({feed_rate_for_GcodeArray, Extrusion_for_GcodeArray,time_for_GcodeArray });
            

            
            // This creates a trajectory that searches around the tool Z and let's the robot move in that null space
            pt = makeTolerancedCartesianPoint(pattern_origin * pose, time_between_points);
            // This creates a trajectory that is rigid. The tool cannot float and must be at exactly this point.
            //pt = makeCartesianPoint(pattern_origin * pose, time_between_points);
            result.push_back(pt);
            publishGoal(myArray[i-seq_element_num+j][6],myArray[i-seq_element_num+j][7],myArray[i-seq_element_num+j][8],time_between_points,myArray[i-seq_element_num+j][2]+ori_adj_x,myArray[i-seq_element_num+j][3]+ori_adj_y,myArray[i-seq_element_num+j][4]+ori_adj_z);
            


        }
        
        int number_of_retun_points = 200 ;
        for (int k=1; k<=number_of_retun_points;k++){
            float unit_distance = (retract_distance/2)/number_of_retun_points;
            float unit_time = (2*time_to_go_back/3)/number_of_retun_points;
            pose = Eigen::Isometry3d::Identity();
            pose.translation() = Eigen::Vector3d(myArray[i-1][2]+ori_adj_x,myArray[i-1][3]+ori_adj_y,myArray[i-1][4]+ori_adj_z+unit_distance*k );
            //pose *= Eigen::AngleAxisd(3.14, Eigen::Vector3d::UnitX()) ;// this flips the tool around so that Z is down
            pose *= Eigen::AngleAxisd(myArray[i-1][6], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(myArray[i-1][7], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(myArray[i-1][8], Eigen::Vector3d::UnitZ());
            //pt = makeCartesianPoint(pattern_origin * pose, 5.0);
            pt = makeTolerancedCartesianPoint(pattern_origin * pose,unit_time);
            result.push_back(pt);
        }
        
        pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(myArray[i-1][2]+ori_adj_x,myArray[i-1][3]+ori_adj_y,myArray[i-1][4]+ori_adj_z+retract_distance );
        //pose *= Eigen::AngleAxisd(3.14, Eigen::Vector3d::UnitX()) ;// this flips the tool around so that Z is down
        pose *= Eigen::AngleAxisd(myArray[i][6], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(myArray[i][7], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(myArray[i][8], Eigen::Vector3d::UnitZ());
        //pt = makeCartesianPoint(pattern_origin * pose, 5.0);
        pt = makeTolerancedCartesianPoint(pattern_origin * pose,time_to_go_back/3);
        

        //GcodeArray.push_back({calculate_F_for_E_and_time(0.05f,0.4f), 0.4f,0.05f }); //extra extrusion for the end.
        GcodeArray.push_back({calculate_F_for_E_and_time(0.05f,-4.0f), -4.0f,0.05f });
        GcodeArray.push_back({calculate_F_for_E_and_time(time_to_go_back -0.05f,-0.5f), -0.5f,time_to_go_back-0.05f });
        result.push_back(pt);
        publishGoal(myArray[i-1][6],myArray[i-1][7],myArray[i-1][8],time_to_go_back,myArray[i-1][2]+ori_adj_x,myArray[i-1][3]+ori_adj_y,myArray[i-1][4]+ori_adj_z+ retract_distance);
        return GcodeArray;
        
        

    }
    std::vector<std::vector<float>> segment_wise_printer_class::init_point(){
        GcodeArray.clear();
        result.clear();
        
        std::vector<double> starting_point_vector = {starting_point[0], starting_point[1], starting_point[2],3.14159265 , 0.0,0.0};
        result = go_to_point(starting_point_vector, 10.0);
        return GcodeArray;
    }
    
    //function to move the robot to a particular joint configuration
    std::vector<std::vector<float>> segment_wise_printer_class::Go_to_joint_pose(std::vector<double> target_joint_pose, double time_in_seconds){
        GcodeArray.clear();
        result.clear();
        //to add the current pose into the trajectory
        result.push_back(descartes_core::TrajectoryPtPtr (new descartes_trajectory::JointTrajectoryPt(joint_pose)));
        result.push_back(descartes_core::TrajectoryPtPtr (new descartes_trajectory::JointTrajectoryPt(target_joint_pose,descartes_core::TimingConstraint(time_in_seconds)) ));

        return GcodeArray;
    }
    
    
    //function to move the robot along a path were each waypoint is defined as joint configuration
    //the path can should be passed as an array of joint configurations and the speed has to be passes in m/seconds
    std::vector<std::vector<float>> segment_wise_printer_class::Follow_joint_pose_path_with_descartes(std::vector<std::vector<double>>  joint_path, float speed){
        GcodeArray.clear();
        result.clear();
        std::vector<double> current_pose_in_loop = {current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z};
        result.push_back(descartes_core::TrajectoryPtPtr (new descartes_trajectory::JointTrajectoryPt(joint_pose)));
        for (size_t i = 0; i < joint_path.size(); ++i) {
    		const std::vector<double>& target_joint_pose = joint_path[i];
    		std::cout<<"working -- 1 " <<std::endl;
		std::vector<double> next_pose_in_loop = segment_wise_printer_class::find_pose_forward_kinematics(target_joint_pose);
		float  distance_x = next_pose_in_loop[0] - current_pose_in_loop[0];
		float  distance_y = next_pose_in_loop[1] - current_pose_in_loop[1];
		float  distance_z = next_pose_in_loop[2] - current_pose_in_loop[2];
		float total_distance = sqrt(pow(distance_x,2)+pow(distance_y,2)+pow(distance_z,2));
		float tbp = total_distance/speed;
		current_pose_in_loop = next_pose_in_loop;
		//to add the current pose into the trajectory
		
		result.push_back(descartes_core::TrajectoryPtPtr (new descartes_trajectory::JointTrajectoryPt(target_joint_pose,descartes_core::TimingConstraint(tbp)) ));
		}
        return GcodeArray;
    }
    
    
    //function to move the robot to a particular pose //give the pose in the format {x,y,z,Rx,Ry,Rz}
    std::vector<std::vector<float>> segment_wise_printer_class::Go_to_cartesian_pose(std::vector<double> target_pose, double time_in_seconds){
        GcodeArray.clear();
        result.clear();
        result = go_to_point(target_pose, time_in_seconds);
        return GcodeArray;
    }


    //function to print a planned Gcode repeatedly based on the user input from the prompt in the terminal
    bool segment_wise_printer_class::Print_agin(std::vector<trajectory_msgs::JointTrajectory> planned_paths, std::vector<std::vector<std::vector<float>>> planned_Gcodes,std::vector<double> target_pose ){
        //note that the target_pose is the Cartesian pose the robot goes to before the paths are planned in the planned_paths
        bool prompt = true;
        //doing the trajectory planning and storing the trajectory
        std::vector<std::vector<float>>  Gcode_array_of_segment;
        trajectory_msgs::JointTrajectory planned_path;


        while (prompt == true) {
            char response;
            std::cout << "Do you want to print the design again? (y/n): ";
            std::cin >> response;

            response = std::tolower(response); // Convert to lowercase for simplicity

            if (response == 'y') {
                std::cout << "You chose yes!" << std::endl;
                prompt = true;
                std::cout << "starting the print now" << std::endl;
                segment_wise_printer_class::update_starting_joint_pose(planned_paths.back());
                Gcode_array_of_segment = segment_wise_printer_class::Go_to_cartesian_pose( target_pose, 10.0);
                planned_path = segment_wise_printer_class::path_planner();
                segment_wise_printer_class::print(planned_path,Gcode_array_of_segment);


                for (size_t i = 0; i < planned_paths.size(); ++i){
                    if (!planned_paths[i].points.empty()) {
                        std::cout<< i<<std::endl;
                        int tester = testExtrusionCalculation(planned_Gcodes[i] );
                        segment_wise_printer_class::print(planned_paths[i],planned_Gcodes[i]);
                    }
                }


            } else if (response == 'n') {
                prompt = false;
                std::cout << "You chose no! exiting the function" << std::endl;
                break;
            } else {
                std::cout << "Invalid input, please enter 'y' or 'n'." << std::endl;
            }
        }

        return true;

    }
    
    trajectory_msgs::JointTrajectory segment_wise_printer_class::path_planner() {
        joint_solution.points.clear();
        descartes_core::RobotModelPtr model(new descartes_moveit::IkFastMoveitStateAdapter());
        //descartes_core::RobotModelPtr model(new descartes_moveit::MoveitStateAdapter);
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
        std::cout<< " status 11" <<std::endl;
        auto f = std::async(std::launch::async, sendGcode , std::ref(Gcode_array_of_segment));
        std::cout<< " status 22" <<std::endl;
        if (!executeTrajectory(joint_solution_to_print , follow_joint_trajectory_action)) {
            ROS_ERROR("Could not execute trajectory!");
            std::cout<< " status 101" <<std::endl;
            }
        std::cout<< " status 33" <<std::endl;
        std::cout<< "printing of segment done"<<std::endl;
    }



















// Definition of rellavant function



// The function used to move the robot to the starting point
std::vector<descartes_core::TrajectoryPtPtr> go_to_point(std::vector<double> point_target,const double time_to_point){
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(
            PLANNING_GROUP);
    //code for moving the robot to the initial position using descartes
    EigenSTL::vector_Isometry3d initial_poses;
    Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();
    init_pose.translation() = Eigen::Vector3d(point_target[0], point_target[1], point_target[2]); //give the desired initial starting pose here!!
    init_pose *= Eigen::AngleAxisd(point_target[3], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(point_target[4], Eigen::Vector3d::UnitY());
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
        descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pattern_origin * init_pose, time_to_point);
        // This creates a trajectory that is rigid. The tool cannot float and must be at exactly this point.
        //descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pattern_origin * init_pose, time_to_point);
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
    
    geometry_msgs::Point point;
    point.x = Px;
    point.y = Py;
    point.z = Pz;
    trajectory_pub.publish(point);
    std::cout<< "planned_point: x  = "<< Px << " , y = "<<Py<< " , z = "<<Pz<<std::endl;
    return 0;
}

//fuction to move the robot driectly using Moveit commander
//since motions using this function does not use Descartes and the Ik_fast Ik solver it can be used to test if descates is working as intented

bool Move_to_pose_with_moveit_cartesian(double X,double Y,double Z,double Rx,double Ry,double Rz){
    ros::NodeHandle node_handle;
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(Rx,Ry,Rz);
    myQuaternion=myQuaternion.normalize();
    std::vector<geometry_msgs::Pose> waypoints;
    
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = myQuaternion.x();
    target_pose1.orientation.y = myQuaternion.y();
    target_pose1.orientation.z = myQuaternion.z();
    target_pose1.orientation.w = myQuaternion.w();
    target_pose1.position.x = X;
    target_pose1.position.y = Y;
    target_pose1.position.z = Z;
    
    waypoints.push_back(target_pose1);
    // Now, we call the planner to compute the plan
    
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,0.01,  // eef_step
                                                 100.0,   // jump_threshold
                                                 trajectory);
  
    ROS_INFO("Visualizing cartesian path (%.2f%% acheived)",
        fraction * 100.0);
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    moveit::core::robotStateToRobotStateMsg(*current_state, display_trajectory.trajectory_start); // Convert RobotStatePtr to RobotStateMsg
    display_trajectory.trajectory.push_back(trajectory);
    display_publisher.publish(display_trajectory);
    
    ROS_INFO("motion will be executed in 15 seconds");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(15.0);
    bool success = false;
    if (fraction > 0.3)
    {
        ROS_INFO("Cartesian path computed successfully. Executing...");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        group.execute(plan);
        success = true;
    }
    else
    {
        ROS_WARN("Only %f of the path was followed.", fraction);
        success = false;
    }    

    
    return success;
    
}

//this function can be used to move to a pose using OMPL as the planner
//it would use the default planner set in moveit usually OMPL unless changed explicitly
bool Move_to_pose_with_moveit(double X,double Y,double Z,double Rx,double Ry,double Rz){
    ros::NodeHandle node_handle;
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(Rx,Ry,Rz);
    myQuaternion=myQuaternion.normalize();
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = myQuaternion.x();
    target_pose1.orientation.y = myQuaternion.y();
    target_pose1.orientation.z = myQuaternion.z();
    target_pose1.orientation.w = myQuaternion.w();
    target_pose1.position.x = X;
    target_pose1.position.y = Y;
    target_pose1.position.z = Z;

    group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    moveit::core::robotStateToRobotStateMsg(*current_state, display_trajectory.trajectory_start); // Convert RobotStatePtr to RobotStateMsg
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);

    ROS_INFO("motion will be executed in 15 seconds");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(15.0);


    group.execute(my_plan);

    return success;

}



//this is the function to display a trajectory planned already in rviz

bool display_planned_path(trajectory_msgs::JointTrajectory& joint_solution_to_print){
    ros::NodeHandle node_handle; 
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    trajectory_msgs::JointTrajectory my_joint_trajectory; // Your trajectory

    // Convert trajectory_msgs::JointTrajectory to moveit_msgs::RobotTrajectory
    moveit_msgs::RobotTrajectory robot_trajectory;
    robot_trajectory.joint_trajectory = my_joint_trajectory;

    // Add your RobotTrajectory to the DisplayTrajectory message
    display_trajectory.trajectory.push_back(robot_trajectory);

    // Publish the DisplayTrajectory message
    display_publisher.publish(display_trajectory);
    
    return true;
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
    if (!GcodeArray_to_print.empty()) {
	    ser.write("G4 S1\r\n");
	    // We are defining the typical command symbols and string elements of a gcode
	    std::string g_str1="G1 F";
	    std::string g_strE=" E";
	    std::string g_strF;
	    std::string g_str2;
	    std::string end_str=" \r\n";
	    std::string g_str;
	    std::string wait_str;
	    std::string wait_str1 = "G4 P";
	    std::string wait_time_str;
	    std::mutex lock;
	    bool motion = false;
	    float z_current;
	    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
	    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
	    z_current = std::round((current_pose.pose.position.z)* 10000.0f) ;
	    while (!motion){
	       current_pose = move_group_interface.getCurrentPose();
	       if ( z_current != std::round(current_pose.pose.position.z * 10000.0f)) {
		   motion = true;
	       }
	       else{
		   sleep(0.0001);
		   //std::cout<<" std::round(current_pose.pose.position.z * 1000.0f)  :  "<<std::round(current_pose.pose.position.z * 1000.0f)<<std::endl;
	       }
	    }
	    for (int j = 0; j < GcodeArray_to_print.size(); j++)
	    {

		if (GcodeArray_to_print[j][1] == 0.0f){
		    //if the E value is 0 then sending the wait command for the specified time 
		    // We take the gcode values from the gcode array and put it inside strings
		    wait_time_str = std::to_string(GcodeArray_to_print[j][2]*1000);
		    wait_str = wait_str1+ wait_time_str+ end_str;
		    // This next command actually sends the gcode to the extruder
		    //ser.write(wait_str);
		}
		else {
		    // We take the gcode values from the gcode array and put it inside strings
		    g_strF = std::to_string(GcodeArray_to_print[j][0]);
		    g_str2=std::to_string(GcodeArray_to_print[j][1]);
		    g_str=g_str1+g_strF+g_strE+g_str2+end_str;

		    // This next command actually sends the gcode to the extruder
		    ser.write(g_str);
		}
		float delay = 0.0;
		// Conversion of the time between points in ms
		std::chrono::milliseconds ms{static_cast<long int>((GcodeArray_to_print[j][2]- delay )*1000)};
		std::this_thread::sleep_for(ms);
	    }
	    std::cout<<" Done sending Gcode to the extruder  : " <<std::endl;
	    }
    return 0;
}

int testExtrusionCalculation(std::vector<std::vector<float>>& GcodeArray_to_print)
{
    float print_time_TBP = 0.0;
    float print_time_from_E = 0.0;
    float F = 0.0;
    float E = 0.0;
    float Time_E = 0.0 ;
    for (int j = 0; j < GcodeArray_to_print.size(); j++)
    {

        // to claculate the total print time as specified by the TBP
        print_time_TBP = print_time_TBP + GcodeArray_to_print[j][2];
        

        // We take the gcode values from the gcode array and put it inside strings
        F = GcodeArray_to_print[j][0];
        E = GcodeArray_to_print[j][1];
        if (F > 0.0f){
            Time_E = calculate_time_for_E_and_F(E,F);
        }
        else{
            Time_E = GcodeArray_to_print[j][2];
        }
        print_time_from_E = print_time_from_E + Time_E;
        
    }
    std::cout<<"print_time_TBP  :  "<<print_time_TBP<<std::endl;
    std::cout<<"print_time_from_E  :  "<<print_time_from_E<<std::endl;
    return 0;
}

//function that can be used to claculate the time for the extrusion of material for -
// a particular E and F value
float calculate_time_for_E_and_F(float E, float F )
{
    E = std::fabs(E);
    float Time_E = (E*60.0f)/F;
    return Time_E;
}

float calculate_F_for_E_and_time(float Time, float E )
{
    E = std::fabs(E);
    float F = (E*60.0f)/Time;
    return F;
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
    return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI / 2000, AxialSymmetricPt::Z_AXIS, TimingConstraint(dt)) );
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
    table.id = name;

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
        if (myArray[i][9] == prev_seg_num and seq_element_num <= 20000 and i < (*count-2)) {
            seq_element_num == seq_element_num++;
            E = myArray[i][5] - E_previous;
            E_subtotal = E_subtotal + E;
            E_previous = myArray[i][5];
            prev_seg_num = myArray[i][9];
        } 
        else if (myArray[i][9] != prev_seg_num or seq_element_num > 20000 or i == (*count-2)) {
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
