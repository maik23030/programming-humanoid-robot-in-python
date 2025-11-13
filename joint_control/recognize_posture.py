'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''
import pickle
import os

from angle_interpolation import AngleInterpolationAgent
from joint_control.keyframes import leftBellyToStand, wipe_forehead
from keyframes import hello


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'

        self.posture_classifier = pickle.load(open('robot_pose.pkl', 'rb'))  # LOAD YOUR CLASSIFIER

        self.classes = sorted(os.listdir('robot_pose_data_json'))


    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        print("Recognized posture:", self.posture)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        # YOUR CODE HERE
        features = [
            perception.joint['LHipYawPitch'],
            perception.joint['LHipRoll'],
            perception.joint['LHipPitch'],
            perception.joint['LKneePitch'],
            perception.joint['RHipYawPitch'],
            perception.joint['RHipRoll'],
            perception.joint['RHipPitch'],
            perception.joint['RKneePitch'],
            perception.imu[0],  # AngleX
            perception.imu[1],  # AngleY
        ]

        # run classifier
        class_id = self.posture_classifier.predict([features])[0]

        # map class id → class name
        posture = self.classes[class_id] if hasattr(self, 'classes') else str(class_id)

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
