#!/usr/bin/env python


import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__' :

    pub = rospy.Publisher('global_path', Path, queue_size=1)
    rospy.init_node('global_map_node', anonymous=True)
    rate = rospy.Rate(5)

    my_path = Path()
    my_path.header.frame_id = "map"

    for i in range(20):
        pose = PoseStamped()
        pose.pose.position.x += 0.1*i
        pose.pose.position.y += 0.1*i
        pose.pose.position.z += 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        #poses.append(pose)
        my_path.poses.append(pose)
    
    while not rospy.is_shutdown():
        #print "hello!"
        pub.publish(my_path)

        rate.sleep()
