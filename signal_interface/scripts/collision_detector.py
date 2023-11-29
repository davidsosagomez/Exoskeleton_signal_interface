#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray


def collision_detector_pub():
    rospy.init_node("collision_detector")
    rospy.Subscriber("/tf_matrix", Float64MultiArray, callback)
    rospy.spin()