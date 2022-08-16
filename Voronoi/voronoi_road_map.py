#Basic functions for math operations and graphs
import math
#from tkinter import TRUE
import numpy as np
import matplotlib.pyplot as plt

#Using the other python file
from dijkstra_search import DijkstraSearch

#Using the Scipy library for Voronoi functions
from scipy.spatial import cKDTree, Voronoi

#Using this to read the custom track layout coordinates
import pandas as pd

show_animation = True


class VoronoiRoadMapPlanner:

    def __init__(self):
        # parameter
        self.N_KNN = 5  # number of edge from one sampled point
        self.MAX_EDGE_LEN = 9.0     # [m] Maximum edge length. 
                                    #Try keeping this as low as possible. The higher the value, the more edges it will traverse.
                                    #This value kind of deepens the search area and hence will also go thru nodes that aren't even on the track

    def planning(self, sx, sy, gx, gy, ox, oy, robot_radius):
        obstacle_tree = cKDTree(np.vstack((ox, oy)).T)

        sample_x, sample_y = self.voronoi_sampling(sx, sy, gx, gy, ox, oy)
        #The below commented section represents the voronoi points or basically the blue dots here
        """if show_animation:  # pragma: no cover
            plt.plot(sample_x, sample_y, ".b")"""

        road_map_info = self.generate_road_map_info(sample_x, sample_y, robot_radius, obstacle_tree)

        rx, ry = DijkstraSearch(show_animation).search(sx, sy, gx, gy, sample_x, sample_y, road_map_info)
        return rx, ry

    def is_collision(self, sx, sy, gx, gy, rr, obstacle_kd_tree):
        x = sx
        y = sy
        dx = gx - sx                    #Differnece between starting point and ending point (X Co-or)
        dy = gy - sy                    #Differnece between starting point and ending point (Y Co-or)
        yaw = math.atan2(dy, dx)        #Returns the value of angle between the given points in the range [-pi,pi]
        d = math.hypot(dx, dy)          #Returns the value of square root of sum of squares of the given numbers

        if d >= self.MAX_EDGE_LEN:
            return True

        D = rr
        n_step = round(d / D)

        for i in range(n_step):
            dist, _ = obstacle_kd_tree.query([x, y])
            if dist <= rr:
                return True  # collision
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)

        # goal point check
        dist, _ = obstacle_kd_tree.query([gx, gy])
        if dist <= rr:
            return True  # collision

        return False  # OK

    def generate_road_map_info(self, node_x, node_y, rr, obstacle_tree):

        road_map = []
        n_sample = len(node_x)
        node_tree = cKDTree(np.vstack((node_x, node_y)).T)

        for (i, ix, iy) in zip(range(n_sample), node_x, node_y):

            dists, indexes = node_tree.query([ix, iy], k=n_sample)

            edge_id = []

            for ii in range(1, len(indexes)):
                nx = node_x[indexes[ii]]
                ny = node_y[indexes[ii]]

                if not self.is_collision(ix, iy, nx, ny, rr, obstacle_tree):
                    edge_id.append(indexes[ii])

                if len(edge_id) >= self.N_KNN:
                    break

            road_map.append(edge_id)

        #  plot_road_map(road_map, sample_x, sample_y)

        return road_map

    @staticmethod
    def plot_road_map(road_map, sample_x, sample_y):  # pragma: no cover

        for i, _ in enumerate(road_map):
            for ii in range(len(road_map[i])):
                ind = road_map[i][ii]

                plt.plot([sample_x[i], sample_x[ind]],[sample_y[i], sample_y[ind]], "-k")

    @staticmethod
    def voronoi_sampling(sx, sy, gx, gy, ox, oy):
        oxy = np.vstack((ox, oy)).T

        # generate voronoi point
        vor = Voronoi(oxy)
        sample_x = [ix for [ix, _] in vor.vertices]
        sample_y = [iy for [_, iy] in vor.vertices]

        sample_x.append(sx)
        sample_y.append(sy)
        sample_x.append(gx)
        sample_y.append(gy)        


        return sample_x, sample_y


def main():

    # start and goal position
    sx = -115.8  # [m]
    sy = 8.1  # [m]
    gx = -57.2  # [m]
    gy = -120.2  # [m]
    robot_size = 1.1  # [m]

    dataset = pd.read_csv('layout2.csv')
    
    ox = dataset.iloc[:, 0].values
    oy = dataset.iloc[:, 1].values
    
    
    #This function shows the starting point and ending point in the graph.
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")      #This line shows the track layout (black points)
        plt.plot(sx, sy, "^r")      #This line shows the starting point
        plt.plot(gx, gy, "^c")      #This line shows the starting point
        plt.grid(False)             #This line enables the grid (the square lines)
        plt.axis("equal")

    rx, ry = VoronoiRoadMapPlanner().planning(sx, sy, gx, gy, ox, oy, robot_size)

    assert rx, 'Cannot find path'

    #This function is to show to red line or basically the path found
    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main()
