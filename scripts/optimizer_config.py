#! /usr/bin/env python3
import rospy
from std_msgs.msg import String


# Initialize the ROS node
rospy.init_node('Post_processor', anonymous=True)




#to control the plotting functions ( to be developed)
plotting_control = True
#change the file locations here for easy access through the dialogue boxes
temporary_file = "../xarm_ws/src/raven/scripts/temp_gcode_file.txt"
optimized_Gcode = "../xarm_ws/src/raven/scripts/Optimized_gcode_file.txt"
log_file= "../xarm_ws/src/raven/scripts/temp_log_file.txt"
traject_file_location = "../Desktop/"
processed_file_location = "../Documents/"
original_traj_file_name = "name"
set_of_recorded_array = []
set_of_error_array = []
set_of_correction_array = []
#to compensate for the inherent shift in the system
fixed_shifts = [0.0,0.0,0.0] #Adjust these numbers if there are constant shift/error in the axis which is always present [x,y,z]

print_status = "unknown"
datalogger_status= "unknown"
optimization_status = "loading"
message = String()
