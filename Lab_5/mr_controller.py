rom ctypes import sizeof
from re import I
import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import rospy
from multiprocessing import Process
from threading import Thread
import time
import pickle


class RobotController:
    def __init__(self, grid_size, nr):
        self.origin = grid_size/2
        self.grid_size = grid_size
        self.grid_map = None
        self.robot_x = 0
        self.robot_y = 0
        self.robot_th = 0
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/PIONIER"+str(nr)+"/scan",
                         LaserScan, self.callback_scan)
        rospy.Subscriber("/PIONIER"+str(nr)+"/RosAria/pose",
                         Odometry, self.callback_position)

    def callback_scan(self, msg):
        scans = list(msg.ranges)
        if self.grid_map is None:
            self.init_map(scans)
        else:
            self.update_map(scans)            
        print("RobotController >> callback scan!")

    def callback_position(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        self.robot_th = euler_from_quaternion(quaternion)[2]

    def calc_pixel(self, hit,i):
        x = int(self.origin + self.robot_x*10\
            + np.cos(self.robot_th+(i)*np.pi/512)*hit*10)
        y = int(self.origin + self.robot_y*10\
            + np.sin(self.robot_th+(i)*np.pi/512)*hit*10)
        return [x, y]

    def init_map(self, scans):
        self.grid_map = np.full((self.grid_size, self.grid_size), 0.5, dtype=float)        
        for i in range(512):
            if (math.isnan(scans[i]) != True):
                if (math.isinf(scans[i]) != True) :
                    scans[i] = self.grid_size/2
                obstacle = self.calc_pixel(scans[i], i)
                self.grid_map[obstacle[0]][obstacle[1]] = 1
                
                points = np.linspace(0,scans[i], int(self.grid_size/2))
                for s in range(len(points)):
                    miss = self.calc_pixel(points[s], i)
                    if (self.grid_map[miss[0]][miss[1]] != 1):
                        self.grid_map[miss[0]][miss[1]] = 0
        
    def update_map(self, scans):
        for i in range(512):
            if (math.isnan(scans[i]) != True):
                if (math.isinf(scans[i]) != True) :
                    scans[i] = self.grid_size/2
                obstacle = self.calc_pixel(scans[i], i)
                self.grid_map[obstacle[0]][obstacle[1]] = 1
                
                points = np.linspace(0,scans[i],int(self.grid_size/2))
                for s in range(len(points)):
                    miss = self.calc_pixel(points[s], i)
                    if (self.grid_map[miss[0]][miss[1]] > 0):
                        self.grid_map[miss[0]][miss[1]] = self.grid_map[miss[0]][miss[1]]/2
        pickle.dump(self.grid_map, open('/tmp/map_file.p', 'wb'))

                # POROWNANIE STAREJ MAPY Z NOWA
    
    def get_map(self):
        print("RobotController >> returning map")        
        return self.grid_map

    def start(self):
        print("RobotController >> STARTED")
        rospy.spin()
