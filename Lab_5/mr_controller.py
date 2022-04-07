from ctypes import sizeof
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
        self.robot_x = None
        self.robot_y = None
        self.robot_th = None
        self.duailsm_map = None
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/PIONIER"+str(nr)+"/scan",
                         LaserScan, self.callback_scan, queue_size=1)
        rospy.Subscriber("/PIONIER"+str(nr)+"/RosAria/pose",
                         Odometry, self.callback_position, queue_size=1)

    def callback_scan(self, msg):
        scans = list(msg.ranges)
        if self.robot_x is None:
            return
        
        if self.grid_map is None:
            self.init_map(scans)
        else:
            self.update_map(scans)  
            self.gen_dualism_map()          
        #print("RobotController >> callback scan!")

    def callback_position(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        self.robot_th = euler_from_quaternion(quaternion)[2]
        #print("Robot: {} {} {}".format(self.robot_x, self.robot_y, self.robot_th))

    def calc_pixel(self, hit, i):
        x = int(self.origin + self.robot_x*10\
            + np.cos(self.robot_th+(i)*np.pi/512-np.pi/2)*hit*10)
            #+ np.cos((i)*np.pi/512)*hit*10)
        y = int(self.origin + self.robot_y*10\
            + np.sin(self.robot_th+(i)*np.pi/512-np.pi/2)*hit*10)
            #+ np.sin((i)*np.pi/512)*hit*10)
        if x < 0 or x > self.grid_size or y < 0 or y > self.grid_size:
            return None
        return [x, y]

    def init_map(self, scans):
        self.grid_map = np.full((self.grid_size, self.grid_size), 0.5, dtype=float)        
        for i in range(512):
            if (math.isinf(scans[i])) or (math.isnan(scans[i])):
                continue
            obstacle = self.calc_pixel(scans[i], i)
            self.grid_map[obstacle[0]][obstacle[1]] = 1
                
            points = np.linspace(0,scans[i], int(self.grid_size*2))
            for s in range(len(points)):
                miss = self.calc_pixel(points[s], i)
                if (self.grid_map[miss[0]][miss[1]] != 1):
                    self.grid_map[miss[0]][miss[1]] = 0
        
    def update_map(self, scans):
        for i in range(512):  # For every meas in
            if (math.isnan(scans[i])):
                continue
            if (math.isinf(scans[i])):
                continue
                scans[i] = 5
            else:
                obstacle = self.calc_pixel(scans[i], i)
                if obstacle is not None:
                    self.grid_map[obstacle[0]][obstacle[1]] = 1
                    if (i == 256):
                      print("X: " + str(self.robot_x))
                      print("H: " + str(scans[i]))
            
            try:
                points = np.linspace(0,scans[i],int(self.grid_size))
                for s in range(len(points)):
                    miss = self.calc_pixel(points[s], i)
                    if miss is not None:                    
                        if (self.grid_map[miss[0]][miss[1]] > 0):
                            self.grid_map[miss[0]][miss[1]] = self.grid_map[miss[0]][miss[1]] * 3/4

            except IndexError:
                continue
            if scans[i] != 5:
                self.grid_map[obstacle[0]][obstacle[1]] = 1

        pickle.dump(self.grid_map, open('/tmp/map_file.p', 'wb'))

                # POROWNANIE STAREJ MAPY Z NOWA
    def gen_dualism_map(self):
        tmp = 0
        self.duailsm_map = np.full((self.grid_size, self.grid_size), 0, dtype=int)     
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                tmp = self.grid_map[i][j]
                if tmp > 0.001:
                    self.duailsm_map[i][j] = 1
                else:
                    self.duailsm_map[i][j] = 0
        # Make obstacles bigger
        x_around = [-1,0,1,0]
        y_around = [0,1,0.-1]
        for a in range(2):
            for i in range(self.grid_size):
                for j in range(self.grid_size):
                    # find Obstacle
                    if (self.duailsm_map[i][j] == 1) or self.duailsm_map[i][j] == 2:
                        # Calc neighbours cells
                        for r in range(3):
                            i_tmp = int(i+x_around[r])
                            j_tmp = int(j+y_around[r])
                            # check if we dont exeed the iterator
                            if (i_tmp > 0) and (i_tmp < self.grid_size) and (j_tmp > 0) and (j_tmp < self.grid_size):
                                if self.duailsm_map[i_tmp][j_tmp] == 0:
                                    self.duailsm_map[i_tmp][j_tmp] = 3
            # normalise map
            for i in range(self.grid_size):
                for j in range(self.grid_size):
                    if self.duailsm_map[i][j] == 3:
                        self.duailsm_map[i][j] = 2
        # Normalise whole map for A*
        for i in range(self.grid_size):
          for j in range(self.grid_size):
              if self.duailsm_map[i][j] > 0:
                  self.duailsm_map[i][j] = 1
        pickle.dump(self.duailsm_map, open('/tmp/duailsm_map_file.p', 'wb'))

    def get_map(self):
        print("RobotController >> returning map")        
        return self.grid_map

    def start(self):
        print("RobotController >> STARTED")
        rospy.spin()
