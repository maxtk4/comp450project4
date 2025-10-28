import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.cm as cm
from matplotlib.colors import Normalize
import numpy as np
import sys
import math

def plot_rectangle(ax, x, y, h, w):

    rect = Rectangle((x, y), w, h,
                     facecolor='gray', alpha=0.3,
                     edgecolor='black', linewidth=1)
    
    ax.add_patch(rect)

def plot_line(ax, x_coords, y_coords, alpha=1, cmap=3):
    colors = cm.plasma(np.linspace(0, 1, 600))

    ax.plot(x_coords, y_coords, color=colors[int(cmap*100)+300], linestyle='--', marker='', alpha=alpha)
           

if __name__ == "__main__":
    # printing out status messages
    print("Test case visualizer for Project 4 Exercises 1 & 2\n...")

    if len(sys.argv) == 3:
        print(f"Visualizing {'pendulum' if sys.argv[2] == '0' else 'car'} geometric solution path\n...")
        print(f"Reading data from file: {sys.argv[1]}")

        fig = plt.figure() # Creates an empty Figure
        ax = fig.add_subplot(111) # Adds a single Axes to the Figure (1 row, 1 column, first subplot)
        ax.set_aspect('equal', adjustable='box') 

        df = pd.read_csv(sys.argv[1], delimiter=' ', header=None)


        ax.set_title(f"Solution Path for {'Pendulum' if sys.argv[2] == '0' else 'Car'} System")
        if sys.argv[2] == '0':
            # get pairs of coordinates and plot the line between them
            for i in range(len(df)-1):
                plot_line(ax, [df.iat[i, 0], df.iat[i+1, 0]], [df.iat[i, 1], df.iat[i+1, 1]])

            print(df.iat[0, 0])
            ax.plot(-1*math.pi/2, 0, marker='o', color='blue', markersize=4)
            ax.text(-1*math.pi/2 + 0.2, 0.2, 'Qstart', fontsize=12, color='blue')

            ax.plot(math.pi/2, 0, marker='o', color='blue', markersize=4)
            ax.text(math.pi/2-0.6, -0.8, 'Qgoal', fontsize=12, color='blue')

            ax.set_ybound(-4.5,6.5)
            ax.set_xbound(-6,5)

            ax.set_xlabel("Theta")
            ax.set_ylabel("Omega")
        if sys.argv[2] == '1':
            # get pairs of coordinates and plot the line between them
            for i in range(len(df)-1):
                plot_line(ax, [df.iat[i, 0], df.iat[i+1, 0]], [df.iat[i, 1], df.iat[i+1, 1]], cmap=df.iat[i, 3])

            fig.colorbar(cm.ScalarMappable(norm=Normalize(vmin=-3, vmax=3), cmap=cm.plasma), ax=ax)

            # plot the start and finish
            ax.plot(-8, -5, marker='o', color='green', markersize=4)
            ax.text(-8 + 0.2, -5 + 0.2, 'Qstart', fontsize=12, color='green')

            ax.plot(8, 5, marker='o', color='green', markersize=4)
            ax.text(8-1, 5-1.2, 'Qgoal', fontsize=12, color='green')

            # plot the obstacles
            plot_rectangle(ax, -10, -10, 4, 20)
            plot_rectangle(ax, 1, -4, 8, 9)
            plot_rectangle(ax, -10, -4, 8, 9)
            plot_rectangle(ax, -10, 6, 4, 20)

            ax.set_ybound(-10,10)
            ax.set_xbound(-10,10)

            ax.set_ylabel("Y Position")
            ax.set_xlabel("X Position")

        plt.show()
    else:
        print("Incorrect command line arguments")