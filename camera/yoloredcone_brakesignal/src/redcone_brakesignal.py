#!/usr/bin/python

import rospy
from random import randint
from std_msgs.msg import Bool


rospy.init_node("This_is_brake_signal")

brakepublisher = rospy.Publisher('/brake_signal', Bool, queue_size=1)
r = rospy.Rate(0.8)

while not rospy.is_shutdown():

    redcone_detect = Bool()
    redcone_detect.data = randint(0,1)

    rospy.loginfo(redcone_detect)

    brakepublisher.publish(redcone_detect)

    r.sleep()
