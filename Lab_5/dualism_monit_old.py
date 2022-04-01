import matplotlib.pyplot as plt
import numpy as np
import time
import pickle


def show_map():
    grid_map = pickle.load(open('/tmp/duailsm_map_file.p', 'rb'))    
    plt.imshow(grid_map, interpolation="nearest", cmap='Blues')
    plt.colorbar()
    plt.draw()
    i = 0
    while True:
        grid_map = pickle.load(open('/tmp/duailsm_map_file.p', 'rb'))
        plt.imshow(grid_map, interpolation="nearest", cmap='Blues')
        plt.draw()
        plt.pause(1/120)
        # print(i)
        # i += 1
        # if i > 500:
        #     return


if __name__ == "__main__":
    show_map()
