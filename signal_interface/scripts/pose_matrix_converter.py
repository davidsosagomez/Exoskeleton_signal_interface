#!/usr/bin/env python3
import rospy
import tf
from unity_robotics_demo_msgs.msg import PosRot
from std_msgs.msg import Float64MultiArray

def pose_callback(pose_msg):
    # Extract position and quaternion from the unity costumize pose message
    position = (pose_msg.pos_x, pose_msg.pos_y, pose_msg.pos_z)
    quaternion = (pose_msg.rot_x, pose_msg.rot_y, pose_msg.rot_z, pose_msg.rot_w)

    # Convert quaternion and position to transformation matrix
    transformation_matrix = tf.transformations.quaternion_matrix(quaternion)
    transformation_matrix[0][3] = position[0]
    transformation_matrix[1][3] = position[1]
    transformation_matrix[2][3] = position[2]

    # Create a Float64MultiArray message to publish the matrix
    matrix_msg = Float64MultiArray()
    matrix_msg.data = transformation_matrix.reshape(-1) # Flatten the matrix to 1D array

    # Publish the transformation matrix
    pub.publish(matrix_msg)

if __name__ == '__main__':
    rospy.init_node('pose_matrix_converter')

    rospy.Subscriber('/pos_rot', PosRot, pose_callback)

    pub = rospy.Publisher('/T_basetracker', Float64MultiArray, queue_size=10)

    rospy.spin()
