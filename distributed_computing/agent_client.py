'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''
import threading
import xmlrpc.client
import weakref

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        def worker():
            try:
                self.proxy.execute_keyframes(keyframes)
            except ReferenceError:
                pass  # ClientAgent was deleted

        t = threading.Thread(target=worker)
        t.daemon = True  # does not block program exit
        t.start()
        return t

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        def worker():
            try:
                self.proxy.set_transform(effector_name, transform)
            except ReferenceError:
                pass

        t = threading.Thread(target=worker)
        t.daemon = True
        t.start()
        return t


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self,host="localhost", port=9000):
        self.post = PostHandler(self)
        url = "http://{}:{}".format(host, port)
        self.rpc = xmlrpc.client.ServerProxy(url, allow_none=True)

        # Provide non-blocking .post interface
        self.post = PostHandler(self)

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.rpc.get_angle(joint_name)

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        return self.rpc.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.rpc.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        return self.rpc.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.rpc.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        return self.rpc.set_transform(effector_name, transform)


if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    # --- Blocking test: get/set angle ---
    print("Testing blocking get_angle / set_angle...")

    print("Setting LHipYawPitch to 0.3 rad...")
    agent.set_angle("LHipYawPitch", 0.3)

    val = agent.get_angle("LHipYawPitch")
    print("Server returned angle =", val)

    # --- Blocking test: transform retrieval ---
    print("\nTesting get_transform...")
    T = agent.get_transform("LLeg")
    print("Transform of LLeg:", T)

    # --- Non-blocking test: set_transform ---
    print("\nTesting non-blocking call: post.set_transform...")
    import time

    thread = agent.post.set_transform("LLeg", [[1, 0, 0, 0],
                                               [0, 1, 0, 0.05],
                                               [0, 0, 1, -0.26],
                                               [0, 0, 0, 1]])

    print("Non-blocking call returned immediately:", thread)
    print("Main thread continues running...")

    # Wait a bit so background thread can finish
    time.sleep(1.0)

    print("Done.")


