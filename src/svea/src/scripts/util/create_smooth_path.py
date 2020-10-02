#!/usr/bin/env python

"""
Module which hold functions to create a path from waypoints.
It uses utilizes dubins path planner.
"""


import math
import numpy as np
import sys
import os

dirname = os.path.dirname(__file__)
dubins_path_planning = os.path.join(dirname,
                                    '../../PythonRobotics/PathPlanning/DubinsPath/')
sys.path.append(os.path.abspath(dubins_path_planning))

import dubins_path_planning as dpp


def create_smooth_path(coords, curvature):
    """
    Creates a smooth path from a set of waypoints.

    :param coords: A list where each index holds the x, y, yaw of the waypoints.
    :type coords: list
    :param curvature: The curvature for the dubins path. 0 - 1.
    :type curvature: float
    :return: cx    A list of x-coordinates for the path
             cy    A list of y-coordinates for the path
             cyaw  A list of yaw-coordinates for the path
    :rtype: cx list, cy list, cyaw list
    """
    cx = []
    cy = []
    cyaw = []

    cx_init, cy_init, yaw_init, _, _ = dpp.dubins_path_planning(coords[0][0],
                                                                coords[0][1],
                                                                coords[0][2],
                                                                coords[1][0],
                                                                coords[1][1],
                                                                coords[1][2],
                                                                curvature)
    cx.extend(cx_init)
    cy.extend(cy_init)
    cyaw.extend(yaw_init)
    i = 1
    cx_init = cx_init[-1]
    cy_init = cy_init[-1]
    yaw_init = yaw_init[-1]
    while i < len(coords)-1:
        cx1, cy1, yaw1, _, _ = dpp.dubins_path_planning(cx_init,
                                                        cy_init,
                                                        yaw_init,
                                                        coords[i+1][0],
                                                        coords[i+1][1],
                                                        coords[i+1][2],
                                                        curvature)
        cx.extend(cx1)
        cy.extend(cy1)
        cyaw.extend(yaw1)
        cx_init = cx1[-1]
        cy_init = cy1[-1]
        yaw_init = yaw1[-1]
        i = i + 1

    return cx, cy, cyaw

def smooth_yaw(yaw):
    """
    Smoothens yaw values.

    :param yaw: A list of yaw values to be smoothened.
    :type yaw: list
    :return: smoothened yaw values
    :rtype: list
    """
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw    