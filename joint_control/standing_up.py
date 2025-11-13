'''In this exercise you need to put all code together to make the robot be able to stand up by its own.

* Task:
    complete the `StandingUpAgent.standing_up` function, e.g. call keyframe motion corresponds to current posture

'''


from recognize_posture import PostureRecognitionAgent
from keyframes.leftBackToStand import leftBackToStand
from keyframes.leftBellyToStand import leftBellyToStand
from keyframes.rightBackToStand import rightBackToStand
from keyframes.rightBellyToStand import rightBellyToStand


class StandingUpAgent(PostureRecognitionAgent):
    def think(self, perception):
        self.standing_up()
        return super(StandingUpAgent, self).think(perception)

    def standing_up(self):
        posture = self.posture
        # YOUR CODE HERE

        # Do nothing if standing
        if posture in ["Stand", "StandInit"]:
            return

        # Choose the stand-up motion
        if posture == "Left":
            # Robot lying on its left side; need to check belly/back
            # Use IMU angle to decide direction
            if self.perception.imu[0] > 0:  # belly
                self.keyframes = leftBellyToStand()
            else:  # back
                self.keyframes = leftBackToStand()

        elif posture == "Right":
            if self.perception.imu[0] > 0:  # belly
                self.keyframes = rightBellyToStand()
            else:
                self.keyframes = rightBackToStand()

        # Belly but centered → choose left version
        elif posture == "Belly":
            self.keyframes = leftBellyToStand()

        # Back but centered → choose left version
        elif posture == "Back":
            self.keyframes = leftBackToStand()

        # Unknown posture → do nothing
        else:
            pass


class TestStandingUpAgent(StandingUpAgent):
    '''this agent turns off all motor to falls down in fixed cycles
    '''
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(TestStandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.stiffness_on_off_time = 0
        self.stiffness_on_cycle = 10  # in seconds
        self.stiffness_off_cycle = 3  # in seconds

    def think(self, perception):
        action = super(TestStandingUpAgent, self).think(perception)
        time_now = perception.time
        if time_now - self.stiffness_on_off_time < self.stiffness_off_cycle:
            action.stiffness = {j: 0 for j in self.joint_names}  # turn off joints
        else:
            action.stiffness = {j: 1 for j in self.joint_names}  # turn on joints
        if time_now - self.stiffness_on_off_time > self.stiffness_on_cycle + self.stiffness_off_cycle:
            self.stiffness_on_off_time = time_now

        return action


if __name__ == '__main__':
    agent = TestStandingUpAgent()
    agent.run()
