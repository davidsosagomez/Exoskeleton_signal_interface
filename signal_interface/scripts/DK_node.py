
#Performing forward knematics for Exoskeleton (converting joint positions to a robot space in cartesian space)
# current joint positions can be accessed from the topic /joint_states
#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import tf


class ForwardKinematics:
    def __init__(self):
        self.jointstate_subscriber = rospy.Subscriber("/joint_states", JointState, self.jointstate_callback)
        self.tf_matrix_pub = rospy.Publisher("/tf_matrix", Float64MultiArray, queue_size = 10)
        self.pose_pub = rospy.Publisher("/end_effector_pose", Pose, queue_size = 10)
        #parameters (taken form https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters)
        self.alpha = np.array([0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2])
        self.a = np.array([0, 0, 0, 0.0825, -0.0825, 0, 0.088])
        self.d = np.array([0.333, 0, 0.316, 0, 0.384, 0, 0.107])

    def jointstate_callback(self, msg):
        self.q = np.array(msg.position, dtype = np.float64)
        #publishing the pose message we calculated using forward kinematics
        pose_in_cartesian, matrix = self.forward_kinematics_solver(self.q)
        self.pose_pub.publish(pose_in_cartesian)
        tf_matrix = Float64MultiArray()  #for future use
        tf_matrix.data = matrix.flatten()
        self.tf_matrix_pub.publish(tf_matrix)

    def dh_between_frames(self, q, a, alpha, d):
        #general formula for transformation between two frames
        tf_matrix_between_frames = np.array([[np.cos(q), -np.sin(q), 0, a],
                                          [np.sin(q)*np.cos(alpha), np.cos(q)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
                                          [np.sin(q)*np.sin(alpha), np.cos(q)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
                                          [0, 0, 0, 1]])
        return tf_matrix_between_frames

    def forward_kinematics_solver(self, q):
        
        for i in range(len(q)):
            if i == 0:
                T_0_7 = self.dh_between_frames(q[i], self.a[i], self.alpha[i], self.d[i])
            else:
                T_0_7 = np.dot(T_0_7, self.dh_between_frames(q[i], self.a[i], self.alpha[i], self.d[i]))
                    
        #Converting tf_matrix to position and orientation to publish as a pose type msg 
        # https://answers.ros.org/question/379109/transformation-matrices-to-geometry_msgspose/
        q = tf.transformations.quaternion_from_matrix(T_0_7)
        pose = Pose()
        pose.position.x = T_0_7[0, 3]
        pose.position.y = T_0_7[1, 3]
        pose.position.z = T_0_7[2, 3]
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose, T_0_7

if __name__ == "__main__":
    rospy.init_node("forward_kinematics")
    fk = ForwardKinematics()
    while not rospy.is_shutdown():
        rospy.spin()