#!/usr/bin/env python

# This script publish the initial position of the robot in our ASCC Smart Home

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped


def main(args):
    global goal_point
    try:
        pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        rospy.init_node('robot_initial_pose')

        initial_pose = PoseWithCovarianceStamped()

        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = -1.07890856266
        initial_pose.pose.pose.position.y = 1.53621780872
        initial_pose.pose.pose.orientation.z = -0.708925242159
        initial_pose.pose.pose.orientation.w = 0.70528363162

        rate = rospy.Rate(10)
        count = 0

        while count < 5:
            pose_pub.publish(initial_pose)
            count += 1
            rate.sleep()

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main(sys.argv)
