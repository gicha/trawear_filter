import numpy as np
import numbers


class MyQuaternion:
    def __init__(self, w_or_q, x=None, y=None, z=None):
        self._q = np.array([1, 0, 0, 0])

        if x is not None and y is not None and z is not None:
            w = w_or_q
            q = np.array([w, x, y, z])
        elif isinstance(w_or_q, MyQuaternion):
            q = np.array(w_or_q.q)
        else:
            q = np.array(w_or_q)
            if len(q) != 4:
                raise ValueError("Expecting a 4-element array or w x y z as parameters")

        self._set_q(q)

    # MyQuaternion specific interfaces

    def conj(self):
        return MyQuaternion(self._q[0], -self._q[1], -self._q[2], -self._q[3])

    def to_angle_axis(self):
        if self[0] == 1 and self[1] == 0 and self[2] == 0 and self[3] == 0:
            return 0, 1, 0, 0
        rad = np.arccos(self[0]) * 2
        imaginary_factor = np.sin(rad / 2)
        if abs(imaginary_factor) < 1e-8:
            return 0, 1, 0, 0
        x = self._q[1] / imaginary_factor
        y = self._q[2] / imaginary_factor
        z = self._q[3] / imaginary_factor
        return rad, x, y, z

    @staticmethod
    def from_angle_axis(rad, x, y, z):
        s = np.sin(rad / 2)
        return MyQuaternion(np.cos(rad / 2), x * s, y * s, z * s)

    def to_euler_angles(self):
        pitch = np.arcsin(2 * self[1] * self[2] + 2 * self[0] * self[3])
        if np.abs(self[1] * self[2] + self[3] * self[0] - 0.5) < 1e-8:
            roll = 0
            yaw = 2 * np.arctan2(self[1], self[0])
        elif np.abs(self[1] * self[2] + self[3] * self[0] + 0.5) < 1e-8:
            roll = -2 * np.arctan2(self[1], self[0])
            yaw = 0
        else:
            roll = np.arctan2(2 * self[0] * self[1] - 2 * self[2] * self[3], 1 - 2 * self[1] ** 2 - 2 * self[3] ** 2)
            yaw = np.arctan2(2 * self[0] * self[2] - 2 * self[1] * self[3], 1 - 2 * self[2] ** 2 - 2 * self[3] ** 2)
        return roll, pitch, yaw

    def to_euler123(self):
        roll = np.arctan2(-2 * (self[2] * self[3] - self[0] * self[1]),
                          self[0] ** 2 - self[1] ** 2 - self[2] ** 2 + self[3] ** 2)
        pitch = np.arcsin(2 * (self[1] * self[3] + self[0] * self[1]))
        yaw = np.arctan2(-2 * (self[1] * self[2] - self[0] * self[3]),
                         self[0] ** 2 + self[1] ** 2 - self[2] ** 2 - self[3] ** 2)
        return roll, pitch, yaw

    def __mul__(self, other):
        if isinstance(other, MyQuaternion):
            w = self._q[0] * other._q[0] - self._q[1] * other._q[1] - self._q[2] * other._q[2] - self._q[3] * other._q[
                3]
            x = self._q[0] * other._q[1] + self._q[1] * other._q[0] + self._q[2] * other._q[3] - self._q[3] * other._q[
                2]
            y = self._q[0] * other._q[2] - self._q[1] * other._q[3] + self._q[2] * other._q[0] + self._q[3] * other._q[
                1]
            z = self._q[0] * other._q[3] + self._q[1] * other._q[2] - self._q[2] * other._q[1] + self._q[3] * other._q[
                0]

            return MyQuaternion(w, x, y, z)
        elif isinstance(other, numbers.Number):
            q = self._q * other
            return MyQuaternion(q)

    def __add__(self, other):
        if not isinstance(other, MyQuaternion):
            if len(other) != 4:
                raise TypeError("MyQuaternions must be added to other quaternions or a 4-element array")
            q = self.q + other
        else:
            q = self.q + other.q

        return MyQuaternion(q)

    def _set_q(self, q):
        self._q = q

    def _get_q(self):
        return self._q

    q = property(_get_q, _set_q)

    def __getitem__(self, item):
        return self._q[item]

    def __array__(self):
        return self._q
