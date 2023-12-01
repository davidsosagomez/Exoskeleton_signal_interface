#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np

class CollisionDetector:
    def __init__(self):
        # Define the predefined matrix for comparison
        self.predefined_matrix = np.array([[-0.55, -0.53, -0.65, -0.39],
                                           [-0.83, 0.45, 0.34, -0.03],
                                           [ 0.11, 0.72, -0.68, 0.47],
                                           [ 0.0, 0.0, 0.0, 1.0]])
        print("Predefined collision matrix: ")
        print(self.predefined_matrix)

        # ROS Node and Publisher setup
        rospy.init_node('collision_detector', anonymous=True)
        self.collision_pub = rospy.Publisher('/collision', Bool, queue_size=10)

        # ROS Subscriber to /tf_matrix
        rospy.Subscriber("/tf_matrix", Float64MultiArray, self.callback)

    def callback(self, msg):
        # Convert data to numpy array
        current_matrix = np.round(np.array(msg.data).reshape(4, 4),2)
        

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
