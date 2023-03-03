#!/usr/bin/env python

import math

"""
This script pre-determines the paths from one area to another (including the charging station).

"""

waypoints= {'(0, 1)': [(1, 1), (1, 3),  (4, 3), (4, 5)],
         '(1, 0)': [(4, 5), (4, 3), (1, 3), (1, 1)],
         '(0, 2)': [(1, 1), (1, 3), (7.5, 3), (7.5, 6.25), (1, 6.25), (1, 7)],
         '(2, 0)': [(1, 7), (1, 6.25), (7.5, 6.25), (7.5, 3), (1, 3), (1, 1)],
         '(0, 3)': [(1, 1), (1, 3), (8, 3), (8, 1), (9, 1), (9, 9), (1, 9)],
         '(3, 0)': [(1, 9), (9, 9), (9, 1), (8, 1), (8, 3), (1, 3), (1, 1)],
         '(1, 2)': [(4, 5), (7.5, 5), (7.5, 6.25), (1, 6.25), (1, 7)],
         '(2, 1)': [(1, 7), (1, 6.25), (7.5, 6.25), (7.5, 5), (4, 5)],
         '(1, 3)': [(4, 5), (4, 3), (8, 3), (8, 1), (9, 1), (9, 9), (1, 9)],
         '(3, 1)': [(1, 9), (9, 9), (9, 1), (8, 1), (8, 3), (4, 3), (4, 5)],
         '(2, 3)': [(1, 7), (1, 6.25), (8, 6.25), (8, 1), (9, 1), (9, 9), (1, 9)],
         '(3, 2)': [(1, 9), (9, 9), (9, 1), (8, 1), (8, 6.25), (1, 6.25), (1, 7)]
         }




def move_to_coords(path):
    """
    This function moves the robot to a target coordinates by a set of waypoints as the path
    :param path: list of waypoints
    :return:
    """
        for coord in path:
            print("Current coordinates:", self.x, self.y, self.theta)
            print("Next waypoint:", coord)
            self.move_to_coords(coord)

        path13 = [(4, 3), (8, 3), (8, 1), (9, 1), (9, 9), (1, 9)]
        for coord in path13:
            print("Current coordinates:", self.x, self.y, self.theta)
            print("Next waypoint:", coord)
            self.move_to_coords(coord)

        path32 = [(1, 9), (9, 9), (9, 1), (8, 1), (8, 6.25), (1, 6.25), (1, 7)]
        for coord in path32:
            print("Current coordinates:", self.x, self.y, self.theta)
            print("Next waypoint:", coord)
            self.move_to_coords(coord)

        path21 = [(1, 7), (1, 6.25), (7.5, 6.25), (7.5, 5), (4, 5)]
        for coord in path21:
            print("Current coordinates:", self.x, self.y, self.theta)
            print("Next waypoint:", coord)
            self.move_to_coords(coord)

        path10 = [(4, 5), (4, 3), (1, 3), (1, 1)]
        for coord in path10:
            print("Current coordinates:", self.x, self.y, self.theta)
            print("Next waypoint:", coord)
            self.move_to_coords(coord)