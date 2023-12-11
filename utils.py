import math
import numpy as np
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

def check_cspace_2x2(x, y):
    deg = list(range(0, 360, 5))
    x_list, y_list = [], []
    for d in deg:
        xl = x + 34 * math.cos(math.radians(d)) 
        yl = y + 34 * math.sin(math.radians(d)) 
        if xl>0 and xl < 800 and yl>0 and yl<300 :
            x_list.append(xl)
            y_list.append(yl)
    return x_list , y_list

def cal_euclidean_dist(rx,ry):
    dist = 0
    for i in range(len(rx)-1):
        point1 = np.array([rx[i], ry[i]])
        point2 = np.array([rx[i+1], ry[i+1]])
        dist = dist + np.linalg.norm(point1 - point2)
    return dist

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def cal_orientation(rx,ry):
    angle_list = []
    vector_list = []
    for i in range(len(rx)-1):
        pointlast = np.array([rx[i], ry[i]])
        pointprev = np.array([rx[i+1], ry[i+1]])
        vector_list.append(pointlast - pointprev)
    for i in range(len(vector_list)-1):
        first_vector = unit_vector(vector_list[i])
        second_vector = unit_vector(vector_list[i+1])
        dot_product = np.dot(first_vector, second_vector)
        angle = np.arccos(dot_product) #angle in radian
        if math.degrees(angle) > 1:
            angle_list.append(int(math.degrees(angle))/5)
    return angle_list