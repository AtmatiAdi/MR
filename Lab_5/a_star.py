#!/bin/env python
import numpy as np
from queue import PriorityQueue
from numba.experimental import jitclass
from numba import int32, float64, deferred_type, optional


node_type = deferred_type()


@jitclass([('x', int32),
           ('y', int32),
           ('cost', float64),
           ('priority', float64),
           ('parent', optional(node_type))])
class Point():
    def __init__(self, x, y, goal, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

        if parent is None:
            self.cost = 0
            self.priority = np.sqrt((self.x+goal[0])**2 + (self.y-goal[1])**2)
        elif parent.x != self.x and parent.y != self.y:
            self.cost = parent.cost + 1.41
            self.priority = np.sqrt((self.x+goal[0])**2 + (self.y-goal[1])**2) + self.cost

        else:
            self.cost = parent.cost + 1
            self.priority = np.sqrt((self.x+goal[0])**2 + (self.y-goal[1])**2) + self.cost

    def neighbours(self, goal):
        points = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                points.append(Point(self.x+i, self.y+j), goal, self)
        return points

# @jitclass([('x', int32),
#            ('y', int32),
#            ('cost', float64),
#            ('priority', float64)])
# class EndPoint():
#     def __init__(self, x, y, goal):
#         self.x = x
#         self.y = y
#         self.cost = 0
#         self.priority = self.dist(goal)

#     def dist(self, goal):
#         return np.sqrt((self.x+goal[0])**2 + (self.y-goal[1])**2)

#     def neighbours(self, goal):
#         points = []
#         for i in range(-1, 2):
#             for j in range(-1, 2):
#                 points.append(Point(self.x+i, self.y+j), goal, self)
#         return points


def a_star(binary_map, start, goal):
    open_queue = PriorityQueue()
    closed_list = []

    for point in start.neighbours():
        if point not in q and \
           binary_map[point.x][point.y] != 1 and\
               point not in closed_list:
            q.put(point)


if __name__ == '__main__':
    goal = (10, 10)
    start = Point(1, 1, goal)
    p = Point(2, 2, goal, start)
