import numpy as np
import matplotlib.pyplot as plt

def sort_vertices(polygon):
    """Sorts vertices by polar angles.

    Args:
        polygon (list[list[float, float]]): list of polygon vertices

    Returns:
        list[list[float, float]]: list of polygon vertices sorted
    """
    cx, cy = polygon.mean(0)  # center of mass
    x, y = polygon.T
    angles = np.arctan2(y-cy, x-cx)
    indices = np.argsort(angles)
    return polygon[indices]


def crossprod(p1, p2):
    """Cross product of two vectors in 2R space.

    Args:
        p1 (list[float, float]): first vector
        p2 (list[float, float): second vector

    Returns:
        float: value of cross product
    """
    return p1[0] * p2[1] - p1[1] * p2[0]


def minkowskisum(pol1, pol2):
    """Calculate Minkowski sum of two convex polygons.

    Args:
        pol1 (np.ndarray[float, float]): first polygon
        pol2 (np.ndarray[float, float]): second polygon

    Returns:
        np.ndarray[np.ndarray[float, float]]: list of the Minkowski sum vertices
    """
    msum = []
    pol1 = sort_vertices(pol1)
    pol2 = sort_vertices(pol2)

    # sort vertices so that is starts with lowest y-value
    min1, min2 = np.argmin(pol1[:, 1]), np.argmin(pol2[:, 1])  # index of vertex with min y value
    pol1 = np.vstack((pol1[:min1], pol1[min1:]))
    pol2 = np.vstack((pol2[:min2], pol2[min2:]))

    i, j = 0, 0
    l1, l2 = len(pol1), len(pol2)
    # iterate through all the vertices
    while i < len(pol1) or j < len(pol2):
        msum.append(pol1[i % l1] + pol2[j % l2])
        cross = crossprod((pol1[(i+1) % l1] - pol1[i % l1]), pol2[(j+1) % l2] - pol2[j % l2])
        # using right-hand rule choose the vector with the lower polar angle and iterate this polygon's vertex
        if cross >= 0:
            i += 1
        if cross <= 0:
            j += 1

    return np.array(msum)

def main():
    # create two polygons
    polygon1 = np.array([[-3, 5], [-3, 1]])
    polygon2 = np.array([[2, 1], [4, 1], [4, 3], [2, 3]])

    # calculate minkowski sum
    msum = minkowskisum(polygon1, polygon2)-3

    # plot the polygons
    polygon1 = sort_vertices(polygon1)
    polygon2 = sort_vertices(polygon2)
    plt.figure(figsize=(7, 4))
    plt.fill(polygon1[:, 0], polygon1[:, 1], color='gray')
    plt.fill(polygon2[:, 0], polygon2[:, 1], color='darkgray')
    plt.fill(msum[:, 0], msum[:, 1])
    ax = plt.gca()
    ax.set_aspect('equal')
    ax.set_axisbelow(True)
    ax.grid()
    plt.show()


if __name__ == '__main__':
  main()