import matplotlib.pyplot as plt
import numpy as np
import time
import pickle
from os.path import exists

path_file = '/tmp/path_file.p'

def show_map():
    fig, ax = plt.subplots()        
    grid_map = pickle.load(open('/tmp/map_file.p', 'rb'))
    ax.imshow(grid_map, interpolation="nearest", cmap='Blues')
    # ax.colorbar()
    plt.draw()
    i = 0
    while True:
        grid_map = pickle.load(open('/tmp/map_file.p', 'rb'))
        ax.imshow(grid_map, interpolation="nearest", cmap='Blues')

        if exists(path_file):
            path = pickle.load(open(path_file, 'rb'))        
            ax.plot([p[1] for p in path], [p[0] for p in path])

        plt.draw()
        plt.pause(2)


if __name__ == "__main__":
    show_map()
