import numpy as np
import pinocchio as pin

class HeadedPDController(object):
    def __init__(self, head, P, D, traj_ref=None):
        self.head = head
        self.traj_ref = traj_ref
        self.P, self.D = P, D

        self.q = self.head.get_sensor('joint_positions')
        self.dq = self.head.get_sensor('joint_velocities')

        self.done = False

    def warmup(self, thread_head):
        thread_head.ti = 0
        self.update(0)

    def update(self, n):
        self.head.read()
        self.q_ref = np.copy(self.traj_ref[n])
        self.dq_ref = np.zeros(3)

    def run(self, thread_head):
        if thread_head.ti < len(self.traj_ref):
            n = thread_head.ti 
        else: 
            n = -1
            self.done = True

        self.update(n)

        tau = self.P * (self.q_ref - self.q) + self.D * (self.dq_ref - self.dq)
        self.head.set_control('ctrl_joint_torques', tau)

