import matplotlib.pyplot as plt
import numpy as np
import time
import pickle
from os.path import exists



def show_map():
    path_line_plot = None
    path_file = '/tmp/path_file.p'
    robot_file = '/tmp/robot_file.p'
    goal_file = '/tmp/goal_file.p'    
    fig, ax = plt.subplots()        
    grid_map = pickle.load(open('/tmp/map_file.p', 'rb'))

    ax.imshow(grid_map, interpolation="nearest", cmap='Blues')
    # ax.colorbar()
    plt.draw()
    i = 0
    while True:
        grid_map = pickle.load(open('/tmp/map_file.p', 'rb'))
        ax.imshow(grid_map, interpolation="nearest", cmap='Blues')

        # if exists(path_file):
        #     if path_line_plot:
        #         ax.lines.pop()
        #     path = pickle.load(open(path_file, 'rb'))

        #     # path changes color every 2 seconds, it's not a bug, it's a feature!
        #     path_line_plot = ax.plot([p[1] for p in path], [p[0] for p in path]# , color='C0' 
        #     )

        if exists(robot_file):
            robot_pos = pickle.load(open(robot_file, 'rb'))

            robot_pos_plot = ax.plot(robot_pos[0], robot_pos[1], marker="^", markersize=10, color=(1,0,0))

        if exists(goal_file):
            goal_pos = pickle.load(open(goal_file, 'rb'))

            goal_pos_plot = ax.plot(goal_pos[0], goal_pos[1], marker="*", markersize=10, color=(1,0.5,0))        
            
        plt.draw()
        plt.pause(2)


if __name__ == "__main__":
    show_map()
