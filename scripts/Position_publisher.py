#! /usr/bin/env python3
import rospy
#from std_msgs.msg import Float32MultiArray
import moveit_commander
from  geometry_msgs.msg import PoseStamped
import copy

#this is the version 2 of the code
rospy.init_node('endeffector_position_publisher')
pub = rospy.Publisher('/eff_position_pub', PoseStamped, queue_size=0)
rate = rospy.Rate(2)
a = PoseStamped()

# remapin the topic for sawyer_gazebo
group_name = "xarm7"
joint_state_topic = ['joint_states:=/robot/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
move_group = moveit_commander.MoveGroupCommander(group_name)


while not rospy.is_shutdown():
    current_pose = move_group.get_current_pose()
    # Get the current joint values
    current_joint_values = move_group.get_current_joint_values()
    #print("current pose : ",current_pose)
    a = copy.deepcopy(current_pose)
    print ("pose : ", a)
    print ("joint values : ", current_joint_values)
    pub.publish(a)
    rate.sleep
