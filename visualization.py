###############
#  Visualization Code for Project 4
#
#  Authors: Max Kuhlman and Henry Prendergast
###############


import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.cm as cm
from matplotlib.colors import Normalize
import numpy as np
import sys
import math
import networkx as nx
import re

def plot_rectangle(ax, x, y, h, w):

    rect = Rectangle((x, y), w, h,
                     facecolor='gray', alpha=0.3,
                     edgecolor='black', linewidth=1)
    
    ax.add_patch(rect)

def plot_line_cmap(ax, x_coords, y_coords, alpha=1, cmap=-3):
    colors = cm.plasma(np.linspace(0, 1, 600))

    ax.plot(x_coords, y_coords, color=colors[int(cmap*100)+300], linestyle='--', marker='', alpha=alpha)

def plot_line(ax, x_coords, y_coords, color='r', alpha=1):

    return ax.plot(x_coords, y_coords, color=color, linestyle='-', marker='', alpha=alpha)
           

if __name__ == "__main__":
    # printing out status messages
    print("Test case visualizer for Project 4 Exercises 1 & 2\n...")

    if len(sys.argv) == 3:
        # print(f"Reading data from file: {sys.argv[1]}")

        fig = plt.figure() # Creates an empty Figure
        ax = fig.add_subplot(111) # Adds a single Axes to the Figure (1 row, 1 column, first subplot)
        # ax.set_aspect('equal', adjustable='box') 

        df_1 = pd.read_csv(sys.argv[1], delimiter=' ', header=None)
        # df_2 = pd.read_csv("./pendulumPathRGRRT_5.txt", delimiter=' ', header=None)
        # df_3 = pd.read_csv("./pendulumPathRGRRT_10.txt", delimiter=' ', header=None)


        ax.set_title(f"Pendulum Graph Data with RGRRT")
        if sys.argv[2] == '0':
            ax.plot(-1*math.pi/2, 0, marker='o', color='black', markersize=4)
            ax.text(-1*math.pi/2 + 0.2, 0.2, 'Qstart', fontsize=12, color='black')

            ax.plot(math.pi/2, 0, marker='o', color='black', markersize=4)
            ax.text(math.pi/2-0.6, -0.8, 'Qgoal', fontsize=12, color='black')

            graph = nx.read_graphml("./rgrrt-pendulum.graphml")

            for u, v, data in graph.edges(data=True):
                node0_coordinates = [float(x) for x in graph.nodes[u]['coords'].split(',')]
                node1_coordinates = [float(x) for x in graph.nodes[v]['coords'].split(',')]

                
                cmap = -3+(6/graph.number_of_edges()*int(re.search(r"\d+", data['id']).group(0)))

                plot_line(ax, [node0_coordinates[0], node1_coordinates[0]],[node0_coordinates[1], node1_coordinates[1]], color='gray', alpha=0.2+10/graph.number_of_edges())

            # # get pairs of coordinates and plot the line between them
            for i in range(len(df_1)-1):
                line1, = plot_line(ax, [df_1.iat[i, 0], df_1.iat[i+1, 0]], [df_1.iat[i, 1], df_1.iat[i+1, 1]], color='r')

            # for i in range(len(df_2)-1):
            #     line2, = plot_line(ax, [df_2.iat[i, 0], df_2.iat[i+1, 0]], [df_2.iat[i, 1], df_2.iat[i+1, 1]], color='g')

            # for i in range(len(df_3)-1):
            #     line3, = plot_line(ax, [df_3.iat[i, 0], df_3.iat[i+1, 0]], [df_3.iat[i, 1], df_3.iat[i+1, 1]], color='b')


            ax.set_ybound(-6,9)
            ax.set_xbound(-4,4)

            # ax.legend([line1, line2, line3], ['Torque = 3', 'Torque = 5', 'Torque = 10'])

            ax.set_xlabel("Theta")
            ax.set_ylabel("Omega")
        if sys.argv[2] == '1':
            # get pairs of coordinates and plot the line between them
            for i in range(len(df_1)-1):
                plot_line_cmap(ax, [df_1.iat[i, 0], df_1.iat[i+1, 0]], [df_1.iat[i, 1], df_1.iat[i+1, 1]], cmap=df_1.iat[i, 3])

            fig.colorbar(cm.ScalarMappable(norm=Normalize(vmin=-3, vmax=3), cmap=cm.plasma), ax=ax, label="Velocity")

            graph = nx.read_graphml("./Graphml Files/rrt-car.graphml")


            for edge, data in graph.edges(data=True):
                node0_coordinates = [float(x) for x in graph.nodes[edge[0]]['coords'].split(',')]
                node1_coordinates = [float(x) for x in graph.nodes[edge[1]]['coords'].split(',')]

                cmap = -3+(6/graph.number_of_edges()*re.search(r"\d+", data['id']))

                plot_line_cmap(ax, [node0_coordinates[0], node1_coordinates[0]],[node0_coordinates[1], node1_coordinates[1]], cmap='gray', alpha=0.25)


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