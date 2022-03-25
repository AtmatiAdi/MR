import matplotlib.pyplot as plt
import numpy as np
import time
import pickle

class MapPlotter:
    def show_map(self, grid_map):
        plt.imshow(grid_map, interpolation="nearest", cmap='Blues')
        plt.colorbar()
        plt.show()
        plt.pause(1/30)

    def update_map(self, grid_map): 
        plt.imshow(grid_map, interpolation="nearest", cmap='Blues')
        plt.show()
        plt.pause(1/30)
