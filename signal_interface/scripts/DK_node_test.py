#!/usr/bin/env python3
import numpy as np
import tf

class ForwardKinematics:
    def __init__(self):
        # DH parameters
        #self.alpha = np.array([np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0, 0])
        #self.a = np.array([0, 0.086, 0.2195, 0.2195, 0.2695, 0.20365])
        #self.d = np.array([0.10293, -0.14106, 0, 0, 0, 0])

        self.alpha = np.array([np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0, 0])
        self.a = np.array([0.0005, 0.086, 0.2195, 0.2195, 0.2695, 0.20365])
        self.d = np.array([0.10293, -0.27756, 0.0697, 0.0142, -0.0133, 0.0252])

    def dh_between_frames(self, q, a, alpha, d):
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

        q = tf.transformations.quaternion_from_matrix(T_0_6)
        position = (T_0_6[0, 3], T_0_6[1, 3], T_0_6[2, 3])
        orientation = (q[0], q[1], q[2], q[3])
        return position, orientation

if __name__ == "__main__":
    fk = ForwardKinematics()
    q_test = np.zeros(6, dtype=np.float64)  # 6 zeros for six joints
    position, orientation = fk.forward_kinematics_solver(q_test)

    print("Position:", position)
    print("Orientation:", orientation)
