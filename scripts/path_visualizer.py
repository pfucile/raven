#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

path = Path()
path_pub = rospy.Publisher('/path', Path, queue_size=10)
counter = 0
def pose_callback(data):
    
    global path,counter
    if len(path.poses)>100000:
        path = Path()
    counter = counter + 1
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose
    path.header = data.header
    path.poses.append(pose)
    
    if (counter  == 50):
        path_pub.publish(path)
        counter = 0
        

def main():
    rospy.init_node('path_node')
    
    odom_sub = rospy.Subscriber('/eff_position_pub', PoseStamped, pose_callback)
    
    
    rospy.spin()

if __name__ == '__main__':
    main()
