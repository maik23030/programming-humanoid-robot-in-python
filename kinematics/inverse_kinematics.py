'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''

import numpy as np
from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def __init__(self):
        super(ForwardKinematicsAgent, self).__init__()

        #From FK
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll',
                                'LElbowYaw', 'LElbowRoll', 'LWristYaw'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll',
                                'RElbowYaw', 'RElbowRoll', 'RWristYaw'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch',
                                'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch',
                                'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       }

    def inverse_kinematics(self, effector_name, target_T,
                           max_iters=60, epsilon=1e-4, step_size=0.5):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE

        joint_names = self.chains[effector_name]  # <-- automatically get FK chain
        dof = len(joint_names)

        # initial guess = current joint positions
        q = np.array([self.joint_positions[j] for j in joint_names], dtype=float)

        for _ in range(max_iters):

            # Forward kinematics at current joint angles
            T = self._fk_from_angles(effector_name, joint_names, q)

            # Position error
            pos_err = target_T[:3, 3] - T[:3, 3]

            # Rotation error via rotation vector
            R_err = target_T[:3, :3] @ T[:3, :3].T
            rot_err = self._rotation_matrix_to_vector(R_err)

            # Full 6×1 error vector
            err = np.hstack([pos_err, rot_err])

            # Convergence
            if np.linalg.norm(err) < epsilon:
                break

            # Jacobian (6×dof)
            J = self._numeric_jacobian(effector_name, joint_names, q)

            # Damped least-squares inverse
            λ = 0.1
            J_pinv = J.T @ np.linalg.inv(J @ J.T + λ * np.eye(6))

            # Update
            q += step_size * (J_pinv @ err)

        joint_angles = q.tolist()

        return joint_angles

    def _numeric_jacobian(self, effector_name, joint_names, q, h=1e-5):
        dof = len(joint_names)
        J = np.zeros((6, dof))

        base_T = self._fk_from_angles(effector_name, joint_names, q)

        for i in range(dof):
            dq = np.zeros_like(q)
            dq[i] = h

            T2 = self._fk_from_angles(effector_name, joint_names, q + dq)

            # position derivative
            dp = (T2[:3, 3] - base_T[:3, 3]) / h

            # rotation derivative
            dR = T2[:3, :3] @ base_T[:3, :3].T
            dω = self._rotation_matrix_to_vector(dR) / h

            J[:, i] = np.hstack([dp, dω])

        return J

    def _rotation_matrix_to_vector(self, R):
        angle = np.arccos(max(min((np.trace(R) - 1) / 2, 1.0), -1.0))
        if abs(angle) < 1e-6:
            return np.zeros(3)

        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) / (2 * np.sin(angle))

        return axis * angle

    def _fk_from_angles(self, effector_name, joint_names, q):
        """Temporarily inject q into joint_positions to compute FK."""
        saved = self.joint_positions.copy()

        for name, angle in zip(joint_names, q):
            self.joint_positions[name] = angle

        T = self.get_transform(effector_name)
        self.joint_positions = saved
        return T

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
