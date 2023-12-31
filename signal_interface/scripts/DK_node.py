
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
        #self.alpha = np.array([np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0, 0])
        #self.a = np.array([0, 0.086, 0.2195, 0.2195, 0.2695, 0.20365])
        #self.d = np.array([0.09793, -0.14106, 0.0697, 0.0692, -0.0133, 0.0252])

        #IN 3D
        self.alpha = np.array([np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0, 0])
        self.a = np.array([0.0005, 0.086, 0.2195, 0.2195, 0.2695, 0.20365])
        self.d = np.array([0.10293, -0.27756, 0.0697, 0.0142, -0.0133, 0.0252])

        #IN 2D
        #self.alpha = np.array([np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0, 0])
        #self.a = np.array([0, 0.086, 0.2195, 0.2195, 0.2695, 0.20365])
        #self.d = np.array([0.09797, -0.14106, 0, 0, 0, 0])

    def jointstate_callback(self, msg):
        self.q = np.array(msg.position, dtype = np.float64)
        # Inserting a 0 at the beginning of the q array
        self.q = np.insert(self.q, 0, 0)
        print("q: ", self.q)
        #publishing the pose message we calculated using forward kinematics
        pose_in_cartesian, matrix = self.forward_kinematics_solver(self.q)
        self.pose_pub.publish(pose_in_cartesian)
        tf_matrix = Float64MultiArray()  #for future use
        tf_matrix.data = matrix.flatten()
        self.tf_matrix_pub.publish(tf_matrix)

    def dh_between_frames(self, q, a, alpha, d):
        #general formula for transformation between two frames
        tf_matrix_between_frames = np.array([[np.cos(q), -np.cos(alpha)*np.sin(q), np.sin(alpha)*np.sin(q), a*np.cos(q)],
                                          [np.sin(q), np.cos(q)*np.cos(alpha), -np.sin(alpha)*np.cos(q), a*np.sin(q)],
                                          [0, np.sin(alpha), np.cos(alpha), d],
                                          [0, 0, 0, 1]])
        return tf_matrix_between_frames

    def forward_kinematics_solver(self, q):
        
        for i in range(len(q)):
            if i == 0:
                T_0_6 = self.dh_between_frames(q[i], self.a[i], self.alpha[i], self.d[i])
            else:
                T_0_6 = np.dot(T_0_6, self.dh_between_frames(q[i], self.a[i], self.alpha[i], self.d[i]))
                    
        #Converting tf_matrix to position and orientation to publish as a pose type msg 
        # https://answers.ros.org/question/379109/transformation-matrices-to-geometry_msgspose/
        q = tf.transformations.quaternion_from_matrix(T_0_6)
        pose = Pose()
        pose.position.x = T_0_6[0, 3]
        pose.position.y = T_0_6[1, 3]
        pose.position.z = T_0_6[2, 3]
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose, T_0_6

if __name__ == "__main__":
    rospy.init_node("forward_kinematics")
    fk = ForwardKinematics()
    while not rospy.is_shutdown():
        rospy.spin()