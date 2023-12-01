#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np

class CollisionDetector:
    def __init__(self):
        # Define the predefined matrix for comparison
        self.predefined_matrix = np.array([[0.95085546, 0.29124849, 0.10511047, 0.301],
                                           [0.19840159, -0.83371212, 0.51532603, -0.139],
                                           [0.2377198, -0.46914648, -0.85052388, 0.266]]).T.reshape(-1, 1)

        # ROS Node and Publisher setup
        rospy.init_node('collision_detector', anonymous=True)
        self.collision_pub = rospy.Publisher('/collision', Bool, queue_size=10)

        # ROS Subscriber to /tf_matrix
        rospy.Subscriber("/tf_matrix", Float64MultiArray, self.callback)

    def callback(self, msg):
        # Convert data to numpy array
        current_matrix = np.array(msg.data).T.reshape(-1, 1)

        # Compare matrices and publish collision status
        if np.array_equal(current_matrix, self.predefined_matrix):
            self.collision_pub.publish(True)
        else:
            self.collision_pub.publish(False)

if __name__ == '__main__':
    try:
        detector = CollisionDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
