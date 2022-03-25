from ctypes import sizeof
from re import I
import numpy as np
import math
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# import rospy
#from numba import jit
#from numba.experimental import jitclass
from monit import MapPlotter
from multiprocessing import Process
from threading import Thread
import time


class RobotController:
    def __init__(self, grid_size, nr):
        self.origin = grid_size/2
        self.grid_size = grid_size
        self.grid_map = np.full((grid_size, grid_size), 0.5, dtype=float)
        self.tmp_grid_map = np.full((grid_size, grid_size), 0.5, dtype=float)
        self.robot_x = 0
        self.robot_y = 0
        self.robot_th = 0
        self.plotter = MapPlotter()
        self.plotter.show_map(self.grid_map)
        # rospy.init_node('listener', anonymous=True)
        # rospy.Subscriber("/PIONIER"+str(nr)+"/scan",
        #                  LaserScan, self.callback_scan)
        # rospy.Subscriber("/PIONIER"+str(nr)+"/RosAria/pose",
        #                  Odometry, self.callback_position)

    def callback_scan(self, msg):
        scans = list(msg.ranges)
        self.init_map(scans)

    def callback_position(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        #self.robot_th = euler_from_quaternion(quaternion)[2]

    def calc_pixel(self, hit,i):
        x = int(self.origin + self.robot_x*10\
            + np.cos(self.robot_th+(i)*np.pi/512)*hit*10)
        y = int(self.origin + self.robot_y*10\
            + np.sin(self.robot_th+(i)*np.pi/512)*hit*10)
        return [x, y]

    def init_map(self, scans):
        for i in range(512):
            if (math.isinf(scans[i]) != True) and (math.isnan(scans[i]) != True):
                obstacle = self.calc_pixel(scans[i], i)
                self.grid_map[obstacle[0]][obstacle[1]] = 1
                
                points = np.linspace(0,scans[i],int(self.grid_size/2))
                for s in range(len(points)):
                    miss = self.calc_pixel(points[s], i)
                    if (self.grid_map[miss[0]][miss[1]] != 1):
                        self.grid_map[miss[0]][miss[1]] = 0
        
    def update_map(self, scans):
        for i in range(512):
            if (math.isinf(scans[i]) != True) and (math.isnan(scans[i]) != True):
                obstacle = self.calc_pixel(scans[i], i)
                self.tmp_grid_map[obstacle[0]][obstacle[1]] = 1
                
                points = np.linspace(0,scans[i],int(self.grid_size/2))
                for s in range(len(points)):
                    miss = self.calc_pixel(points[s], i)
                    if (self.tmp_grid_map[miss[0]][miss[1]] != 1):
                        self.tmp_grid_map[miss[0]][miss[1]] = 0
                # POROWNANIE STAREJ MAPY Z NOWA
    
    def get_map(self):
        print("RobotController >> returning map")   
        return self.grid_map

    def start(self):
        print("RobotController >> STARTED")
        while True:
            self.plotter.update_map(self.grid_map)
        # rospy.spin()

