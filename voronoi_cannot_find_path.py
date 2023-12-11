"""

Voronoi Road Map Planner

https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/VoronoiRoadMap

"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree, Voronoi
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))
from dijkstra_search import DijkstraSearch

show_animation = True


class VoronoiRoadMapPlanner:

    def __init__(self):
        # parameter
        self.N_KNN = 10  # number of edge from one sampled point
        self.MAX_EDGE_LEN = 30.0  # [m] Maximum edge length

    def planning(self, sx, sy, gx, gy, ox, oy, robot_radius):
        obstacle_tree = cKDTree(np.vstack((ox, oy)).T)

        sample_x, sample_y = self.voronoi_sampling(sx, sy, gx, gy, ox, oy)
        if show_animation:  # pragma: no cover
            plt.plot(sample_x, sample_y, ".b")

        road_map_info = self.generate_road_map_info(
            sample_x, sample_y, robot_radius, obstacle_tree)

        rx, ry = DijkstraSearch(show_animation).search(sx, sy, gx, gy,
                                                       sample_x, sample_y,
                                                       road_map_info)
        return rx, ry

    def is_collision(self, sx, sy, gx, gy, rr, obstacle_kd_tree):
        x = sx
        y = sy
        dx = gx - sx
        dy = gy - sy
        yaw = math.atan2(gy - sy, gx - sx)
        d = math.hypot(dx, dy)

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
        """
        Road map generation

        node_x: [m] x positions of sampled points
        node_y: [m] y positions of sampled points
        rr: Robot Radius[m]
        obstacle_tree: KDTree object of obstacles
        """

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

                plt.plot([sample_x[i], sample_x[ind]],
                         [sample_y[i], sample_y[ind]], "-k")

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
        
def check_cspace(x, y):
    deg = list(range(0, 360, 5))
    x_list, y_list = [], []
    for d in deg:
        xl = x + 16 * math.cos(math.radians(d)) 
        yl = y + 16 * math.sin(math.radians(d)) 
        if xl>0 and xl < 400 and yl>0 and yl<150 :
            x_list.append(xl)
            y_list.append(yl)
    return x_list , y_list

def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 25.0  # [m]
    sy = 25.0  # [m]
    gx = 750.0/2  # [m]
    gy = 25.0  # [m]
    #grid_size = 1.0  # [m]
    robot_size = 12.5*math.sqrt(2)  # [m]

    # set obstacle positions
    ox, oy = [], []
    deg = list(range(0, 360, 5))
    for i in range(0, 400):
        # ox.append(i)
        # oy.append(0.0)
        for d in deg:
            xl = i + 16 * math.cos(math.radians(d)) 
            yl = 0.0 + 16 * math.sin(math.radians(d)) 
            if xl>0 and xl < 400 and yl>0 and yl<150 and yl == 16 and xl > 16 and xl < 384:
                ox.append(xl)
                oy.append(yl)
    #right
    for i in range(0, 150):
        # ox.append(400.0)
        # oy.append(i)
        for d in deg:
            xl = 400.0 + 16 * math.cos(math.radians(d)) 
            yl = i + 16 * math.sin(math.radians(d)) 
            if xl>0 and xl < 400 and yl>0 and yl<150 and xl == 400 - 16 and yl > 16 and yl < 134:
                ox.append(xl)
                oy.append(yl)
    for i in range(0, 401):
        # ox.append(i)
        # oy.append(150.0)
        for d in deg:
            xl = i + 16 * math.cos(math.radians(d)) 
            yl = 150.0 + 16 * math.sin(math.radians(d)) 
            if xl>0 and xl < 400 and yl>0 and yl<150 and yl == 150 - 16 and xl > 16 and xl < 384:
                ox.append(xl)
                oy.append(yl)
    for i in range(0, 151):
        # ox.append(0.0)
        # oy.append(i)
        for d in deg:
            xl = 0.0 + 16 * math.cos(math.radians(d)) 
            yl = i + 16 * math.sin(math.radians(d)) 
            if xl>0 and xl < 400 and yl>0 and yl<150 and xl == 16 and yl > 16 and yl < 134:
                ox.append(xl)
                oy.append(yl)
    #obstacles
    #additional line segment
    
    for i in range(0, 16):
        ox.append(50.0 - 16 + i)              
        oy.append(99.0)
        ox.append(150.0 + i)
        oy.append(99.0)
        ox.append(200.0)
        oy.append(50.0 - 16 + i)
        ox.append(350.0 + i)
        oy.append(99.0)
        ox.append(250.0 - 16 + i)
        oy.append(99.0)
    #left obstacle
    for i in range(0, 100):
        if i > 15:
            ox.append(99.0)
            oy.append(i)
        for d in deg:
            xl = 99.0 + 16 * math.cos(math.radians(d)) 
            yl = i + 16 * math.sin(math.radians(d)) 
            if xl>0 and xl < 400 and yl>0 and yl<150 and (xl == 99 + 16 or xl == 99 - 16) and yl > 16 and yl < 83:
                ox.append(xl)
                oy.append(yl)
    for i in range(0, 100):
        ox.append(50.0 + i)
        oy.append(99.0)
        for d in deg:
            xl = 50.0 + i + 16 * math.cos(math.radians(d)) 
            yl = 99.0 + 16 * math.sin(math.radians(d)) 
            if xl>0 and xl < 400 and yl>0 and yl<150 and yl == 99 + 16 or yl == 99 - 16:
                ox.append(xl)
                oy.append(yl)
    #left half circle
    deg2 = list(range(90, 270, 5))
    for d2 in deg2:
        xl = 50.0 + 16 * math.cos(math.radians(d2)) 
        yl = 99.0 + 16 * math.sin(math.radians(d2))  
        ox.append(xl)
        oy.append(yl)
    deg3 = list(range(90, -90, -5))
    #right half circle
    for d3 in deg3:
        xl = 150.0 + 16 * math.cos(math.radians(d3)) 
        yl = 99.0 + 16 * math.sin(math.radians(d3))  
        ox.append(xl)
        oy.append(yl)     
    #mid obstacles
    for i in range(0, 100):
        if 50 + i < 150-16:
            ox.append(200.0)
            oy.append(50.0 + i)
        for d in deg:
            xl = 200.0 + 16 * math.cos(math.radians(d)) 
            yl = 50.0 + i + 16 * math.sin(math.radians(d)) 
            if xl>0 and xl < 400 and yl>0 and yl<150 and (xl == 200 + 16 or xl == 200 - 16) and yl < 134:
                ox.append(xl)
                oy.append(yl)
    deg4 = list(range(180, 360, 5))
    for d4 in deg4:
        xl = 200.0 + 16 * math.cos(math.radians(d4)) 
        yl = 50.0 + 16 * math.sin(math.radians(d4))  
        ox.append(xl)
        oy.append(yl)            
    ox.append(200.0)
    oy.append(50.0 - 16)
    #right obstacle
    for i in range(0, 100):
        if i > 15:
            ox.append(299.0)
            oy.append(i)
        for d in deg:
            xl = 299.0 + 16 * math.cos(math.radians(d)) 
            yl = i + 16 * math.sin(math.radians(d)) 
            if xl>0 and xl < 400 and yl>0 and yl<150 and (xl == 299 + 16 or xl == 299 - 16) and yl > 16 and yl < 83:
                ox.append(xl)
                oy.append(yl)
    for i in range(0, 100):
        ox.append(250.0 + i)
        oy.append(99.0)
        for d in deg:
            xl = 250.0 + i + 16 * math.cos(math.radians(d)) 
            yl = 99.0 + 16 * math.sin(math.radians(d)) 
            if xl>0 and xl < 400 and yl>0 and yl<150 and yl == 99 + 16 or yl == 99 - 16:
                ox.append(xl)
                oy.append(yl)
    #left half circle
    for d2 in deg2:
        xl = 250.0 + 16 * math.cos(math.radians(d2)) 
        yl = 99.0 + 16 * math.sin(math.radians(d2))  
        ox.append(xl)
        oy.append(yl)
    #right half circle
    for d3 in deg3:
        xl = 350.0 + 16 * math.cos(math.radians(d3)) 
        yl = 99.0 + 16 * math.sin(math.radians(d3))  
        ox.append(xl)
        oy.append(yl)     


    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "^r")
        plt.plot(gx, gy, "^c")
        plt.grid(True)
        plt.gca().invert_yaxis()
        plt.axis("equal")

    rx, ry = VoronoiRoadMapPlanner().planning(sx, sy, gx, gy, ox, oy,
                                              robot_size)

    assert rx, 'Cannot found path'

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.1)
        plt.show()


if __name__ == '__main__':
    main()
