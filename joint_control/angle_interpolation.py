'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''
from joint_control.keyframes import wipe_forehead, leftBellyToStand
from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = None

    def think(self, perception):

        # initialize start time
        if self.start_time is None:
            self.start_time = perception.time

        current_time = perception.time - self.start_time

        target_joints = self.angle_interpolation(self.keyframes, current_time)

        #target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        else:
            target_joints['RHipYawPitch'] = 0.0


        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)


    def bezier_interp(self, t, p0, p1, p2, p3):
        """Cubic Bezier interpolation."""
        return (
                (1 - t) ** 3 * p0
                + 3 * (1 - t) ** 2 * t * p1
                + 3 * (1 - t) * t ** 2 * p2
                + t ** 3 * p3
        )


    def angle_interpolation(self, keyframes, current_time):
        target_joints = {}
        # YOUR CODE HERE

        names, times, keys = keyframes

        if not names:
            return {}


        target_joints = {}

        for i, name in enumerate(names):
            joint_times = times[i]
            joint_keys = keys[i]

            # find segment containing current_time
            if current_time <= joint_times[0]:
                target_joints[name] = joint_keys[0][0]
                continue
            if current_time >= joint_times[-1]:
                target_joints[name] = joint_keys[-1][0]
                continue

            # find keyframe interval
            for j in range(1, len(joint_times)):
                if joint_times[j] >= current_time:
                    t0 = joint_times[j - 1]
                    t1 = joint_times[j]
                    key0 = joint_keys[j - 1]
                    key1 = joint_keys[j]

                    angle0 = key0[0]
                    angle1 = key1[0]

                    # handle data: [int type, float dTime, float dAngle]
                    handle_out = key0[2]  # handle after current point
                    handle_in = key1[1]  # handle before next point

                    # compute control points
                    p0 = angle0
                    p1 = angle0 + handle_out[2]  # outgoing handle
                    p2 = angle1 + handle_in[2]  # incoming handle
                    p3 = angle1

                    # normalize local time (0–1)
                    t_norm = (current_time - t0) / (t1 - t0)
                    target_joints[name] = self.bezier_interp(t_norm, p0, p1, p2, p3)
                    break

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = wipe_forehead(motion=None)  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
