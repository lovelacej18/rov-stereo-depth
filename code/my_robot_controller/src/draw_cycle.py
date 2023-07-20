#!/usr/bin/env python3
import rospy
# using a new package means we need to update the package.xml with the new dependency!
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("draw_cycle")
    rospy.loginfo("Node has been started")


    # use same name as topic
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        # publish cmd_vel
            # create a message
        msg = Twist()
        msg.linear.x = 2.0 # positive - foward. 
        msg.angular.z = 1.0 # we're in 2D so angular.x and y do not exit
                # don't need to fill out all fields - they go to default values

            # send msg to publisher 
        pub.publish(msg)

        rate.sleep()