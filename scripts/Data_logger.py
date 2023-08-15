#! /usr/bin/env python3
import rospy
import numpy as np
import matplotlib
matplotlib.use('TKAgg') #set the backend to TkAgg
from matplotlib import pyplot as plt
from std_msgs.msg import String
from  geometry_msgs.msg import Pose
from  geometry_msgs.msg import PoseStamped
from numpy import sqrt
from datetime import datetime
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import threading
import sys
import time
import moveit_commander


lock = threading.Lock()
#arrays for storing the data
act_traj = np.array([],dtype=float)
act_joint_traj = np.array([],dtype=float)
goal_traj= np.array([],dtype=float)
goal_joint_traj= np.array([],dtype=float)


goal_array = np.empty(((0, (4))),dtype=float) #defining numpy arrays for storing the goal data
motion = False # logic used to start the recording when the robot starts moving
z_motion = [] # array used to start the recording when the robot starts moving
z_motion_sub = None
sub_goal = None
current_pose = None

joint_state_topic = ['joint_states:=/robot/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
sawyer_group = moveit_commander.MoveGroupCommander("xarm7")
dirname =  "../Documents/" # set the folder for saving the log file
message1= "data logger running, The log data is being stored as txt files in the folder - "
print(message1+ dirname)
time_now = 0
print_time  = 0
start_time = 0


def status_callback(msg):
    global  log_file,sub_goal,motion,z_motion_sub,time_now,print_time,start_time
    if msg.data == "start":
        print("The planning for a new print has started")
        clear_arr()  # clear all the arrays/lists
        Time_stamp= datetime.now().strftime("%Y-%m-%d %H-%M-%S")
        # convert datetime obj to string
        str_time_stamp = str(Time_stamp)
        filename = dirname+str_time_stamp+".txt"
        print("the log data for this print will be stored in the file : " , filename)
        log_file = open(filename, 'w')
        # Subscribe to the goal topic
        sub_goal = rospy.Subscriber('/goal_pub', Pose, callback_goal)
    elif msg.data=="trajectory calculation done":
        z_motion_sub = rospy.Subscriber('/eff_position_pub', PoseStamped, z_motion_callback) #to trigger the logger when the robot is in motion
        write_to_file()  # write the data stored in the goal arrays to the file
        print("The print planning is done")
        print("waiting for execution")
        while motion != True:
            time.sleep ( 0.01)
        if motion == True:
            print("Robot in motion?   ", motion)
            print(" total expected print time :", print_time, "  seconds")
            while time_now<= print_time:
                position_logger()
            if time_now > print_time:
                    time_now = 0
                    print_time  = 0
                    start_time = 0
                    clear_arr()  # clear all the arrays/lists
                    motion = False
                    log_file.close() #closing the log file
                    sub_goal.unregister()
                    print("Done logging data")




def clear_arr():
    with lock:
        global goal_array
        goal_array = np.empty(((0, (4))),dtype=float) #defining numpy arrays for storing the goal data

def write_to_file():
    log_file.write("  goal points   \n")
    log_file.write("      X        Y       Z      TBP      \n")
    for i in range (0,np.size(goal_array,0),1) :
        log_file.write(str(goal_array[i,0])+" "+str(goal_array[i,1])+" "+str(goal_array[i,2])+" "+ str(goal_array[i,3])+"\n")

    log_file.write("  actual points   \n")
    log_file.write("      X        Y       Z      v  \n")
def position_logger():
    with lock:
        global time_now,current_pose,start_time
        current_pose = sawyer_group.get_current_pose()
        #to record the starting time of recording the motion
        if start_time  == 0:
            start_time  = float(float(current_pose.header.stamp.secs)+ (float(current_pose.header.stamp.nsecs)/1000000000))
        #to write the positions to the log file
        log_file.write(str(float(current_pose.pose.position.x))+" "+str(float(current_pose.pose.position.y))+" "+str(float(current_pose.pose.position.z))+" "+ str(float(current_pose.header.stamp.secs)+ (float(current_pose.header.stamp.nsecs)/1000000000))+"\n")
        time_now = (float(float(current_pose.header.stamp.secs)+ (float(current_pose.header.stamp.nsecs)/1000000000))) - start_time
        time.sleep(0.0098) # sleep is to make sure that the Acquisition rate is close to 100Hz as it was found to be the maximum


def z_motion_callback(pose):
    with lock:
        global motion, z_motion,z_motion_sub
        z_motion.append(float(pose.pose.position.z))
        if round(z_motion[0],3) != round(float(pose.pose.position.z),3):
            motion = True
            z_motion_sub.unregister()
            z_motion.clear()
            time.sleep(0.09)
def callback_goal(pose):
    with lock:
        global goal_array,print_time
        goal_array = np.append(goal_array,[[float(pose.position.x), float(pose.position.y),float(pose.position.z), float(pose.orientation.w) ]],axis=0)
        print_time  = print_time + float(pose.orientation.w)
def main():
    rospy.init_node('position_plotter')
    rospy.Subscriber('/print_stat', String, status_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
