#!/bin/env python
import numpy as np
from queue import PriorityQueue


class EndPoint():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0

    def neighbours(self):
        points = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                points.append((self.x+i, self.y+j))
        return points        


class Point():
    def __init__(self, x, y, goal, parent):
        self.x = x
        self.y = y
        self.parent = parent
        if parent.x != self.x and parent.y != self.y:
            self.cost = parent.cost + 1.41
        else:
            self.cost = parent.cost + 1
        self.priority = self.dist(goal) + self.cost

    def dist(self, goal):
        return np.sqrt(self.x+goal.x)**2 + (self.y-goal.y)**2)

    def neighbours(self):
        points = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                points.append(Point(self.x+i, self.y+j), goal, self)
        return points


def a_star(binary_map, start, goal):
    open_queue = PriorityQueue()
    closed_list = []

    for point in start.neighbours():
        if point not in q and \
           binary_map[point.x][point.y] != 1 and\
               point not in closed_list:
            q.put(point)


if __name__ == '__main__':
    start = EndPoint(1, 1)
    end = EndPoint(10, 10)
