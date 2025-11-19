'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from joint_control.recognize_posture import  PostureRecognitionAgent


def rot_x(theta):
    return matrix([[1, 0, 0, 0],
                   [0, np.cos(theta), -np.sin(theta), 0],
                   [0, np.sin(theta), np.cos(theta), 0],
                   [0, 0, 0, 1]])


def rot_y(theta):
    return matrix([[np.cos(theta), 0, np.sin(theta), 0],
                   [0, 1, 0, 0],
                   [-np.sin(theta), 0, np.cos(theta), 0],
                   [0, 0, 0, 1]])


def rot_z(theta):
    return matrix([[np.cos(theta), -np.sin(theta), 0, 0],
                   [np.sin(theta), np.cos(theta), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch']
                       # YOUR CODE HERE

                       'LArm': ['LShoulderPitch', 'LShoulderRoll',
                                'LElbowYaw', 'LElbowRoll', 'LWristYaw'],

                       'RArm': ['RShoulderPitch', 'RShoulderRoll',
                                'RElbowYaw', 'RElbowRoll', 'RWristYaw'],

                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch',
                                'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],

                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch',
                                'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE

        if joint_name == "HeadYaw":
            # neck offset (torso → neck)
            Tx = 0
            Ty = 0
            Tz = 0.1265  # meters

            # translation matrix
            T_trans = matrix([[1, 0, 0, Tx],
                              [0, 1, 0, Ty],
                              [0, 0, 1, Tz],
                              [0, 0, 0, 1]])

            # rotation around z-axis
            T_rot = self.rot_z(joint_angle)

            T = T_trans * T_rot

            # 2. HeadPitch: rotation around y-axis, no translation
        elif joint_name == "HeadPitch":
            T = self.rot_y(joint_angle)

        elif joint_name == "LShoulderPitch":
            # translation from torso -> shoulder
            Tx = 0.0
            Ty = +0.098  # left arm Y offset
            Tz = +0.100  # shoulder height

            T_trans = matrix([[1, 0, 0, Tx],
                              [0, 1, 0, Ty],
                              [0, 0, 1, Tz],
                              [0, 0, 0, 1]])

            # rotation around Y-axis
            T_rot = rot_y(joint_angle)

            T = T_trans * T_rot

        elif joint_name == "LShoulderRoll":
            # no translation between shoulder pitch and roll
            # rotation around X
            T = rot_x(joint_angle)

        elif joint_name == "LElbowYaw":
            # translation from shoulder roll -> elbow yaw
            Tx = 0.105  # upper arm length in meters
            Ty = 0.0
            Tz = 0.0

            T_trans = matrix([[1, 0, 0, Tx],
                              [0, 1, 0, Ty],
                              [0, 0, 1, Tz],
                              [0, 0, 0, 1]])

            # rotation around Z-axis
            T_rot = rot_z(joint_angle)

            T = T_trans * T_rot

        elif joint_name == "LElbowRoll":
            # rotation around X-axis
            # no translation
            T = rot_x(joint_angle)

        elif joint_name == "LWristYaw":
            # translation from elbow roll → wrist yaw
            Tx = 0.056  # forearm length in meters
            Ty = 0.0
            Tz = 0.0

            T_trans = matrix([[1, 0, 0, Tx],
                              [0, 1, 0, Ty],
                              [0, 0, 1, Tz],
                              [0, 0, 0, 1]])

            # wrist yaw rotates around the Z-axis
            T_rot = rot_z(joint_angle)

            T = T_trans * T_rot

        elif joint_name == "RShoulderPitch":
            Tx = 0.0
            Ty = -0.098  # right arm offset is mirrored in Y
            Tz = 0.100

            T_trans = matrix([[1, 0, 0, Tx],
                              [0, 1, 0, Ty],
                              [0, 0, 1, Tz],
                              [0, 0, 0, 1]])

            T_rot = rot_y(joint_angle)
            T = T_trans * T_rot

        elif joint_name == "RShoulderRoll":
            T = rot_x(joint_angle)

        elif joint_name == "RElbowYaw":
            Tx = 0.105
            Ty = 0.0
            Tz = 0.0

            T_trans = matrix([[1, 0, 0, Tx],
                              [0, 1, 0, Ty],
                              [0, 0, 1, Tz],
                              [0, 0, 0, 1]])

            T_rot = rot_z(joint_angle)
            T = T_trans * T_rot

        elif joint_name == "RElbowRoll":
            T = rot_x(joint_angle)

        elif joint_name == "RWristYaw":
            Tx = 0.056
            Ty = 0.0
            Tz = 0.0

            T_trans = matrix([[1, 0, 0, Tx],
                              [0, 1, 0, Ty],
                              [0, 0, 1, Tz],
                              [0, 0, 0, 1]])

            T_rot = rot_z(joint_angle)
            T = T_trans * T_rot

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
