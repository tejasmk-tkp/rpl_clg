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
    'rrt(start, goal, max_iterations, step_size)' : Find path using RRT algorithm.
    'plot_map()' : Plot the environment grid map on matplotlib for visualization.
    'plot_path(path, start, goal)' : Plot the found path on the environment map.
    'plot_rrt_tree(tree, path, start, goal)' : Plot the RRT tree and found path.
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

    def rrt(self, start, goal, max_iterations=5000, step_size=1, goal_sample_rate=0.1):
        """
        Purpose:
        ---
        Find path using RRT (Rapidly-exploring Random Tree) algorithm
        Uses only 4-directional movement (up, down, left, right) - no diagonals

        Input Arguments:
        ---
        'start' : [ <class 'tuple'> ]start - start point of the bot as a tuple of form (row, column)
        'goal' : [ <class 'tuple'> ]goal - end goal of the bot as a tuple of form (row, column)
        'max_iterations' : [ <class 'int'> ] - maximum number of iterations (default: 5000)
        'step_size' : [ <class 'int'> ] - step size for tree extension (default: 1)
        'goal_sample_rate' : [ <class 'float'> ] - probability of sampling goal (default: 0.1)

        Returns:
        ---
        'path' : [<class 'list'>]path - returns a list containing coordinates of the path as tuples of form (row, column) if path found else returns None
        'tree' : [<class 'dict'>]tree - dictionary containing parent-child relationships of the explored tree

        Example call:
        ---
        path, tree = rrt((0, 0), (7, 7))
        """

        def manhattan_distance(p1, p2):
            return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

        def get_nearest_node(tree, random_point):
            min_dist = float('inf')
            nearest = None
            for node in tree.keys():
                dist = manhattan_distance(node, random_point)
                if dist < min_dist:
                    min_dist = dist
                    nearest = node
            return nearest

        def steer_grid(from_node, to_node, step_size):
            # Move toward target using only 4 directions (up, down, left, right)
            moves = []
            row_diff = to_node[0] - from_node[0]
            col_diff = to_node[1] - from_node[1]
            
            # Try to move in the direction with larger difference first
            if abs(row_diff) >= abs(col_diff):
                if row_diff > 0:
                    moves.append((1, 0))  # down
                elif row_diff < 0:
                    moves.append((-1, 0))  # up
                if col_diff > 0:
                    moves.append((0, 1))  # right
                elif col_diff < 0:
                    moves.append((0, -1))  # left
            else:
                if col_diff > 0:
                    moves.append((0, 1))  # right
                elif col_diff < 0:
                    moves.append((0, -1))  # left
                if row_diff > 0:
                    moves.append((1, 0))  # down
                elif row_diff < 0:
                    moves.append((-1, 0))  # up
            
            # Try each move direction
            for dr, dc in moves:
                new_node = (from_node[0] + dr * step_size, from_node[1] + dc * step_size)
                
                # Check bounds
                if (0 <= new_node[0] < self.env_map.shape[0] and 
                    0 <= new_node[1] < self.env_map.shape[1]):
                    # Check if not obstacle
                    if self.env_map[new_node[0], new_node[1]] != 1:
                        return new_node
            
            return None  # No valid move

        # Initialize tree with start node
        tree = {start: None}
        
        for iteration in range(max_iterations):
            # Sample random point (with bias toward goal)
            if np.random.random() < goal_sample_rate:
                random_point = goal
            else:
                random_point = (np.random.randint(0, self.env_map.shape[0]),
                              np.random.randint(0, self.env_map.shape[1]))
            
            # Find nearest node in tree
            nearest_node = get_nearest_node(tree, random_point)
            
            # Steer toward random point (only 4 directions)
            new_node = steer_grid(nearest_node, random_point, step_size)
            
            # If valid move found and not already in tree
            if new_node is not None and new_node not in tree:
                tree[new_node] = nearest_node
                
                # Check if goal is reached
                if new_node == goal:
                    # Reconstruct path
                    path = []
                    current = goal
                    while current is not None:
                        path.append(current)
                        current = tree[current]
                    path.reverse()
                    
                    print(f"Path found in {iteration + 1} iterations!")
                    return path, tree
        
        print("No path found within max iterations!")
        return None, tree

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

    def plot_rrt_tree(self, tree, path, start, goal):
        """
        Purpose:
        ---
        Plot the RRT tree and found path on the gridmap using matplotlib
        
        Input Arguments:
        ---
        'tree' : [ <class 'dict'> ]tree - RRT tree as dictionary of parent-child relationships
        'path' : [ <class 'list'> ]path - found path as a list of tuples
        'start' : [ <class 'tuple'> ]start - start point of the bot as a tuple of form (row, column)
        'goal' : [ <class 'tuple'> ]goal - end goal of the bot as a tuple of form (row, column)

        Example call:
        ---
        plot_rrt_tree(tree, path, start, goal)
        """

        # Plot tree edges
        for node, parent in tree.items():
            if parent is not None:
                self.ax.plot([parent[1], node[1]], [parent[0], node[0]], 
                           color="lightblue", linewidth=0.5, alpha=0.6)
        
        # Plot path if found
        if path is not None:
            rows, cols = zip(*path)
            self.ax.plot(cols, rows, color="red", linewidth=2, marker="o", 
                       markersize=4, label="Path")
        
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

grid_map = np.array([[0, 0, 0, 0, 0, 0, 1, 0],
                     [0, 0, 0, 0, 1, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1, 0, 0],
                     [1, 0, 0, 0, 0, 0, 0, 1],
                     [0, 1, 0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1, 0, 0]
                    ])

myDobot = myBot(grid_map)

start = (0, 0)
goal = (7, 1)

print(f"Obstacle Ratio: {myDobot.obstacle_ratio()}")

# Test A* algorithm
print("\n=== A* Algorithm ===")
path_astar = myDobot.a_star(start, goal)
myDobot.plot_map()
myDobot.plot_path(path_astar, start, goal)
myDobot.show_plot()

# Test RRT algorithm
print("\n=== RRT Algorithm ===")
myDobot.fig, myDobot.ax = plt.subplots()  # Create new figure for RRT
path_rrt, tree_rrt = myDobot.rrt(start, goal, max_iterations=5000, step_size=1)
myDobot.plot_map()
myDobot.plot_rrt_tree(tree_rrt, path_rrt, start, goal)
myDobot.show_plot()
