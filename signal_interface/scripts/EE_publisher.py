#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import tf

class ExoskeletonTracker:
    def __init__(self):
        self.basetracker_subscriber = rospy.Subscriber("/T_basetracker", Float64MultiArray, self.basetracker_callback)
        self.tf_matrix_subscriber = rospy.Subscriber("/tf_matrix", Float64MultiArray, self.tf_matrix_callback)
        self.A_EE_pub = rospy.Publisher("/A_EE", Float64MultiArray, queue_size=10)
        self.pose_EE_pub = rospy.Publisher("/Pose_EE", Pose, queue_size=10)
        
        self.T_basetracker = None
        self.tf_matrix = None

    def basetracker_callback(self, msg):
        self.T_basetracker = np.array(msg.data).reshape(4, 4)
        self.publish_result()

    def tf_matrix_callback(self, msg):
        self.tf_matrix = np.array(msg.data).reshape(4, 4)
        self.publish_result()

    def publish_result(self):
        if self.T_basetracker is not None and self.tf_matrix is not None:
            result_matrix = np.dot(self.T_basetracker, self.tf_matrix)

            # Publish 4x4 matrix
            matrix_msg = Float64MultiArray()
            matrix_msg.data = result_matrix.flatten()
            self.A_EE_pub.publish(matrix_msg)

            # Convert to pose and publish
            q = tf.transformations.quaternion_from_matrix(result_matrix)
            pose = Pose()
            pose.position.x = result_matrix[0, 3]
            pose.position.y = result_matrix[1, 3]
            pose.position.z = result_matrix[2, 3]
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            self.pose_EE_pub.publish(pose)

if __name__ == "__main__":
    rospy.init_node("exoskeleton_tracker")
    et = ExoskeletonTracker()
    rospy.spin()
