import matplotlib.pyplot as plt
import numpy as np
import time
import pickle
from os.path import exists



def show_map():
    path_line_plot = None
    path_file = '/tmp/path_file.p'    
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
            if path_line_plot:
                ax.lines.pop()
            path = pickle.load(open(path_file, 'rb'))

            # path changes color every 2 seconds, it's not a bug, it's a feature!
            path_line_plot = ax.plot([p[1] for p in path], [p[0] for p in path]# , color='C0' 
            )

        plt.draw()
        plt.pause(2)


if __name__ == "__main__":
    show_map()
