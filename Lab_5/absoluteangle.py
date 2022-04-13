import numpy as np
from math import copysign


class PioneerAngle():
    def __init__(self, val):
        if val >= 0:
            pi02_form = val % (2*np.pi)
            if pi02_form > np.pi:
                self.val = -(pi02_form - np.pi)
            else:
                self.val = pi02_form
        else:
            pi02_form = (-val) % (2*np.pi)
            if pi02_form > np.pi:
                self.val = -(pi02_form - np.pi)
            else:
                self.val = - pi02_form

        if self.val == -np.pi:
            self.val = np.pi

    def __sub__(self, other):
        if abs(self.val - other.val) <= np.pi:
            return PioneerAngle(self.val - other.val)
        return PioneerAngle(-copysign(2*np.pi - abs(self.val - other.val), self.val - other.val))

    def __add__(self, other):
        return PioneerAngle(self.val + other.val)
