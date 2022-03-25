import matplotlib.pyplot as plt
import numpy as np
import time
import pickle

class MapPlotter:
    def show_map(self, map):
        plt.imshow(map, interpolation="nearest", cmap='Blues')
        plt.colorbar()
        plt.show()
        while True:
            plt.imshow(map, interpolation="nearest", cmap='Blues')
            plt.show()
            plt.pause(1/30)

