#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(messeage):
    rospy.loginfo("I heard %s", messeage.data)

rospy.init_node('listener')
sub = rospy.Subscriber('chatter',String,callback)
rospy.spin()