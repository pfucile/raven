//
// Created by vivek on 14-2-24.
//
#include <iostream>
#include <vector>



std::vector<std::vector<int>> segmentation_array = find_segments(myArray);// Initialization of the function to find the relevant segments


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


class segment_printer {
private:
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(
            PLANNING_GROUP);
    float GcodeArray[10000][3]
    int seq_element_num
    int i
    std::vector<int> dataArray; // Array to store processed data
    std::vector<double> joint_group_positions_ss;
    std::vector<descartes_core::TrajectoryPtPtr> result;
    geometry_msgs::PoseStamped current_pose

public:
    void get_current_joint_position() {
        moveit::core::RobotStatePtr current_state_ss = move_group_interface.getCurrentState();
        current_state_ss->copyJointGroupPositions(joint_model_group, joint_group_positions_ss);
        std::vector<double> joint_pose = {joint_group_positions_ss[0], joint_group_positions_ss[1], joint_group_positions_ss[2], joint_group_positions_ss[3],joint_group_positions_ss[4], joint_group_positions_ss[5],joint_group_positions_ss[6]};
        current_pose = move_group_interface.getCurrentPose ();
    }
    void get_start_and_stop(const std::vector<int>& input){
        seq_element_num = 1+input[1]-input[0];
        i = 1+input[1]
    }
    void spiral_move_down(){
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

        float  distance = current_pose.pose.position.z - (myArray[i - seq_element_num][4]+ori_adj_z);
        descartes_core::TrajectoryPtPtr pt ;
        int number_of_points =   distance*1000*1;
        float t = 0;
        float Rot_x = M_PI;
        float Rot_y = 0;
        float delta_Rot_x = (myArray[i - seq_element_num][6] - Rot_x)/number_of_points ;
        float delta_Rot_y = (myArray[i - seq_element_num][7] - Rot_y)/number_of_points ;
        float radius = 0.06;
        for (int k=0; k<number_of_points;k++)
        {
            pose = Eigen::Isometry3d::Identity();
            float time_to_point = time_mov_to_start / number_of_points;
            float X_pos = current_pose.pose.position.x + (radius * cos(t)) - radius;
            float Y_pos = current_pose.pose.position.y + (radius * sin(t));
            float Z_pos = current_pose.pose.position.z - ((distance/(2*M_PI)) * t);
            Rot_x = Rot_x + delta_Rot_x;
            Rot_y = Rot_y + delta_Rot_y;
            t = t + (2 * M_PI / number_of_points);
            pose.translation() = Eigen::Vector3d(X_pos , Y_pos , Z_pos );
            pose *= Eigen::AngleAxisd(Rot_x, Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(Rot_y,
                                      Eigen::Vector3d::UnitY()); // this flips the tool around so that Z is down
            //descartes_core::TrajectoryPtPtr pt = makeCartesianPoint(pattern_origin * pose, time_to_point);
            pt = makeTolerancedCartesianPoint(pattern_origin * pose, time_to_point);
            result.push_back(pt);
            publishGoal(myArray[i - seq_element_num][6], myArray[i - seq_element_num][7],
                        myArray[i - seq_element_num][8], time_to_point, X_pos , Y_pos ,
                        Z_pos);
        }

    }
    // Function to process input data and create an array
    void processData(std::vector<int> input) {

        for (int j = 0; j <seq_element_num; j++)
        {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
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
            time_between_points = (sqrt(pow(myArray[i-seq_element_num+j][2]-myArray[i-seq_element_num+j-1][2],2)+pow(myArray[i-seq_element_num+j][3]-myArray[i-seq_element_num+j-1][3],2)+pow(myArray[i-seq_element_num+j][4]-myArray[i-seq_element_num+j-1][4],2))*1000)/print_velocity ;
            GcodeArray[j+3][0] =  (60 *(myArray[i-seq_element_num+j][5] - myArray[i-seq_element_num+j-1][5] ))/ time_between_points ; //calculating the material feed rate in mm/minutes for the extruder
            GcodeArray[j+3][1] =  myArray[i-seq_element_num+j][5] - myArray[i-seq_element_num+j-1][5] ; //for gcodes with Evalue
            GcodeArray[j+3][2] =  time_between_points ;


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

    void planner() {
        //now lets make the plan by passing the trajectory to the planner
        if (!planner->planPath(result))
        {
            ROS_ERROR("Could not solve for a valid path");
            return -3;
        }
        // now we have to extract the calculated path
        std::vector<descartes_core::TrajectoryPtPtr> plan_result;
        if (!planner->getPath(plan_result))
        {
            ROS_ERROR("Could not retrieve path");
            return -4;
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
            return -5;
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
            return -6;
        }
    }
};



