'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes

        # BLOCK until keyframes are finished
        while self.keyframes is not None:
            self.step()  # one simulation/control loop step

        return True

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        # Ensure FK is up to date
        self.forward_kinematics(self.joint_positions)

        T = self.transforms.get(name, None)
        if T is None:
            return None

        # Convert matrix to list-of-lists for RPC serialization
        return T.tolist()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        # Convert transform to numpy array
        import numpy as np
        target_T = np.array(transform, dtype=float)

        # Compute IK
        q = self.inverse_kinematics(effector_name, target_T)

        joint_names = self.chains[effector_name]
        times = [0.5] * len(q)

        # Create blocking keyframe command
        self.keyframes = (joint_names, times, q)

        return True

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

