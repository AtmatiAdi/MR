#!/bin/env python
import numpy as np
from queue import PriorityQueue
from numba.experimental import jitclass
from numba import int32, float64, deferred_type, optional
from numba import jit
import time
import pickle


node_type = deferred_type()


# @jitclass([('x', int32),
#            ('y', int32),
#            ('cost', float64),
#            ('priority', float64),
#            ('parent', optional(node_type))])
class Point():
    def __init__(self, x, y, goal, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        print("generated {} {}".format(x, y))
        if parent is None:
            self.cost = 0
        elif parent.x != self.x and parent.y != self.y:
            self.cost = parent.cost + 1.41

        else:
            self.cost = parent.cost + 1

        self.priority = np.sqrt((self.x+goal[0])**2 + (self.y-goal[1])**2) + self.cost

    def __lt__(self, other):
        return (self.priority < other.priority)
            
    def neighbours(self, goal):
        points = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                points.append(Point(self.x+i, self.y+j, goal, self))
        return points

    def same_cords(self, point):
        if point.x == self.x and point.y == self.y:
            return True
        return False

    def is_part_of(self, some_list):
        for point in some_list:
            if self.same_cords(point):
                if self.priority >= point.priority:
                    return True
        return False


# node_type.define(Point.class_type.instance_type)


# @jit(nopython=True)
def a_star(binary_map, start, goal):
    open_queue = PriorityQueue()
    open_list = []
    closed_list = []

    open_queue.put(start)
    open_list.append(start)

    while not open_queue.empty():
        q = open_queue.get()
        open_list.remove(q)
        for point in q.neighbours(goal):
            if point.x == goal[0] and point.y == goal[1]:
                return point

            if binary_map[point.x][point.y] == 1:
                continue
            
            if point.is_part_of(open_list):
                continue

            if point.is_part_of(closed_list):
                continue

            open_queue.put(point)
            open_list.append(point)
        closed_list.append(q)


if __name__ == '__main__':
    goal = (45, 45)
    start = Point(50, 50, goal)

    bmap = np.full((100, 100), 0, dtype=int)
    fmap = np.full((100, 100), 0, dtype=int)
    point = a_star(bmap, start, goal)
    while True:
        if point.x == start.x and point.y == start.y:
            break
        fmap[point.x][point.y] = 1
        print("path {} {}".format(point.x, point.y))
        point = point.parent
    pickle.dump(fmap, open('/tmp/map_file.p', 'wb'))
