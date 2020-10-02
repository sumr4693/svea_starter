#!/usr/bin/env python

"""
Module for bicycle models with params set for SVEA cars.
"""

import numpy as np
import math
import matplotlib.pyplot as plt

__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se"
__status__ = "Development"


class SimpleBicycleState(object):
    """
    Extended from PythonRobotics for compatibility. State-space
    equation is:

        .. math::

            \\dot{x}     &= v\\cos(\\phi), \\\\
            \\dot{y}     &= v\\sin(\\phi), \\\\
            \\dot{\\phi}  &= \\frac{v}{L} \\tan(\\delta), \\\\
            \\dot{v}     &= a.

    where :math:`x, y, \phi, v` are x position, y position, yaw,
    and velocity; :math:`\delta, a` are the steering angle and
    acceleration inputs; and :math:`L` is the wheel
    base length. This object also includes a method for dynamics updates
    based on the set sampling time and the embedded bicycle model.
    Units are [m, rad, s, m/s]

    :param x: Initial x position, defaults to 0.0
    :type x: float
    :param y: Initial y position, defaults to 0.0
    :type y: float
    :param yaw: Initial yaw, defaults to 0.0
    :type yaw: float
    :param v: Initial velocity, defaults to 0.0
    :type v: float
    :param dt: Sampling time for dynamics update, defaults to 0.1
    :type dt: float
    """

    L = 0.32
    DELTA_MAX = np.radians(30.0)  # max steering angle [rad]

    TAU = 0.1 # gain for simulating SVEA's ESC

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.1):
        """ Initialize state. """

        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        self.dt = dt

    def _normalize_angle(self, angle):
        """ Normalize an angle to [-pi, pi]. """
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def _build_param_printout(self):
        return ("## Simple Bicycle State:\n"
                + "  - x: {0}".format(self.x)
                + "  - y: {0}".format(self.y)
                + "  - yaw: {0}".format(self.yaw)
                + "  - v: {0}".format(self.v)
                + "  - dt: {0}".format(self.dt))

    def __repr__(self):
        return self._build_param_printout()
    def __str__(self):
        return self._build_param_printout()

    def _sim_esc(self, target_velocity):
        # simulates esc dynamics
        return 1/self.TAU * (target_velocity - self.v)

    def _update(self, accel, delta):
        # update state using simple bicycle model dynamics

        delta = np.clip(delta, -self.DELTA_MAX, self.DELTA_MAX)

        self.x += self.v * np.cos(self.yaw) * self.dt
        self.y += self.v * np.sin(self.yaw) * self.dt
        self.yaw += self.v / self.L * np.tan(delta) * self.dt
        self.yaw = self._normalize_angle(self.yaw)
        self.v += accel * self.dt

    def update(self, steering, velocity,
                     transmission = 0,
                     differential_front = 0,
                     differential_rear = 0,
                     ctrl_code = 0):
        """
        Updates state using set sampling time, dt, and embedded bicycle
        dynamics. Designed to take same inputs as SVEA vehicle's
        low-level interface.

        :param steering: input steering angle for the car
        :type steering: float
        :param velocity: input velocity for the car
        :type velocity: float
        :param transmission: [deprecated]
        :param differential_front: [deprecated]
        :param differential_rear: [deprecated]
        :param ctrl_code: [deprecated]
        """

        accel = self._sim_esc(velocity)
        delta = steering

        self._update(accel, delta)

    def get_state(self):
        """
        (will be updated to use properties)

        :return: State as numpy array [x, y, yaw, v]
        :rtype: numpy.ndarray
        """
        state = [self.x,
                 self.y,
                 self.yaw,
                 self.v]
        return np.asarray(state)

    def get_readable_state(self):
        """
        Returns state as self-sufficiently readable state
        (will be updated to use properties)

        :return: State as readable dict {x: __, y: __, yaw: __, v: __}
        :rtype: dict
        """
        state = {"x": self.x,
                 "y": self.y,
                 "yaw": self.yaw,
                 "v": self.v}
        return state

    def get_state_dim(self):
        """
        (will be updated to use properties)

        :return: Dimension of state space
        :rtype: int
        """
        return len(self.get_state())

    def set_dt(self, dt):
        """
        Setter for sampling time used in dynamics update
        (will be updated to use properties)
        """
        self.dt = dt

    def get_dt(self):
        """
        (will be updated to use properties)

        :return: Currently set sampling time
        :rtype: float
        """
        return self.dt
