"""
# Author: Tejas M K
# Filename: a_star_algo.py
# Classes: myBot()
"""

import numpy as np
import matplotlib.pyplot as plt
import heapq


class myBot:
    '''
    Purpose:
    ---
    Class to initialize a robot object to find path using a start algorithm in a given environmental grid map.

    Attributes:
    ---
    'env_map' : <class 'numpy.ndarray'> 
        A 2D list representing the map/grid.

    Methods:
    ---
    'a_star(start, goal)' : Find the most optimal path using a star algorithm.
    'plot_map()' : Plot the environment grid map on matplotlib for visualization.
    'plot_path(path, start, goal)' : Plot the found path on the environment map.
    'show_plot()' : Display the path on the environment grid map using matplotlib.
    'obstacle_ratio()' : Calculate the ratio of number cells with to obstacles to total number of cells in the environment grid map.
    
    Example initalization:
    ---
    bot = myBot(env_map)
    '''

    def __init__(self, env_map):
        self.env_map = env_map
        self.fig, self.ax = plt.subplots()  # figure/axis for batch plotting

    def a_star(self, start, goal):
        """
        Purpose:
        ---
        Find optimal path using a* algorithm using manhattan heuristics

        Input Arguments:
        ---
        'start' : [ <class 'tuple'> ]start - start point of the bot as a tuple of form (row, column)
        'goal' : [ <class 'tuple'> ]goal - end goal of the bot as a tuple of form (row, column)

        Returns:
        ---
        'path' : [<class 'list'>]path - returns a list containing coordinates of the path as tuples of form (row, column) if path found else returns None

        Example call:
        ---
        a_star((0, 0), (3, 3))
        """

        def manhattan(node, goal):  # helper function to calculate heuristics
            return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

        open_list = [
        ]  # store nodes with potential to be the most optimal path
        heapq.heappush(open_list, (0, start))
        closed_list = set()  # store explored nodes
        parent_nodes = {}  # store parent child relations for explored nodes
        g_cost = {start: 0}  # store cost to each node from start

        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4 directions

        while open_list:
            f_current, current = heapq.heappop(open_list)

            if (
                    current == goal
            ):  # if goal node found backtrack route using parent_nodes dictionary
                path = []

                while current in parent_nodes:
                    path.append(current)
                    current = parent_nodes[current]

                path.append(start)
                path.reverse()

                return path

            closed_list.add(current)

            for dr, dc in moves:  # explore adjacent nodes
                neighbor = (current[0] + dr, current[1] + dc)

                if (0 <= neighbor[0] < self.env_map.shape[0]
                        and 0 <= neighbor[1] < self.env_map.shape[1]):
                    if (self.env_map[neighbor[0], neighbor[1]] == 1
                            or neighbor in closed_list):
                        continue

                    g = g_cost[current] + 1
                    h = manhattan(neighbor, goal)
                    f = g + h

                    if neighbor not in g_cost or g < g_cost[neighbor]:
                        g_cost[neighbor] = g
                        parent_nodes[neighbor] = current
                        heapq.heappush(open_list, (f, neighbor))

        print("No path found!")
        return None

    def plot_map(self):
        """
        Purpose:
        ---
        Plot the environment grid map using matplotlib

        Example call:
        ---
        plot_map()
        """

        self.ax.imshow(self.env_map,
                       cmap="Greys",
                       origin="upper",
                       interpolation="none")
        self.ax.set_xticks(np.arange(-0.5, self.env_map.shape[1], 1))
        self.ax.set_yticks(np.arange(-0.5, self.env_map.shape[0], 1))
        self.ax.set_xticklabels([])
        self.ax.set_yticklabels([])
        self.ax.grid(True, color="black", linewidth=1)
        self.ax.invert_yaxis()  # row 0 on top


    def plot_path(self, path, start, goal):
        """
        Purpose:
        ---
        Plot the found path on the gridmap using matplotlib,
        highlighting start and goal nodes.
        
        Input Arguments:
        ---
        'path' : [ <class 'list'> ]path - found path as a list of tuples
        'start' : [ <class 'tuple'> ]start - start point of the bot as a tuple of form (row, column)
        'goal' : [ <class 'tuple'> ]goal - end goal of the bot as a tuple of form (row, column)

        Example call:
        ---
        plot_path(path, start, goal)
        """

        if path is None:
            print("No path to plot!")
            return

        rows, cols = zip(*path)
        # Plot the path
        self.ax.plot(cols, rows, color="red", linewidth=2, marker="o", markersize=4)

        # Highlight start (green) and goal (blue)
        self.ax.scatter(start[1], start[0], color="green", s=100, label="Start", zorder=5)
        self.ax.scatter(goal[1], goal[0], color="blue", s=100, label="Goal", zorder=5)

        # Add a legend
        self.ax.legend(loc="upper right")
    
    def show_plot(self):
        """
        Purpose:
        ---
        Visualize the environment grid map and the found path using matplotlib

        Example call:
        ---
        show_plot()
        """

        print("press 'q' to exit")
        plt.show()

    def obstacle_ratio(self):
        """
        Purpose:
        ---
        Calculate obstacle to total cells ratio in the enviroment

        Returns:
        ---
        `ratio` : [ <class 'numpy.float64'> ]ratio Returns the ratio of obstacles present to total number of cells in the environment map

        Example call:
        ---
        obstacle_ratio()"""

        total_cells = self.env_map.size  # total number of cells

        num_obstacles = np.sum(
            self.env_map == 1)  # count of cells with 1 (obstacle)

        ratio = num_obstacles / total_cells

        return ratio


# implementation

grid_map = np.array([[0, 0, 0, 0, 0, 1],
                     [0, 1, 0, 1, 0, 0],
                     [1, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 1, 0],
                     [0, 0, 1, 0, 0, 1],
                     [0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 1, 0],
                     [0, 0, 0, 1, 0, 0]
                    ])

myDobot = myBot(grid_map)

start = (0, 0)
goal = (7, 5)
path = myDobot.a_star(start, goal)

print(f"Obstacle Ratio: {myDobot.obstacle_ratio()}")

myDobot.plot_map()  # draw the grid
myDobot.plot_path(path, start, goal)  # overlay path
myDobot.show_plot()  # show the path on the environment map
