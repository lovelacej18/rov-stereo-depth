#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose

def pose_callback(msg: Pose):
    rospy.loginfo(f"({0}, {1})".format(str(msg.x), str(msg.y)))

if __name__ == '__main__':
    rospy.init_node("turtle_post_subscriber")
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)

    rospy.loginfo("Node has been started")

    # an infinite loop while roscore is running
    # usually the last line of your program
    rospy.spin()