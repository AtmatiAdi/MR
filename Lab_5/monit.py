import matplotlib.pyplot as plt
import numpy as np
import time
import pickle
from mr_controller import RobotController


class MapPlotter:
    def show_map(self, ctrl):
        plt.imshow(ctrl.get_map(), interpolation="nearest", cmap='Blues')
        plt.colorbar()
        plt.draw()
        plt.pause(1/300)        
        while True:
            plt.imshow(ctrl.get_map(), interpolation="nearest", cmap='Blues')
            plt.draw()
            plt.pause(1/300)

