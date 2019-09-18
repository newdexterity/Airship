from . import Segment
import numpy as np
import matplotlib.pyplot as plt


class Path(object):

    def __init__(self, segment_list, dp):
        """
        Initialises Path.
        Args:
            segment_list(list[Segment.Segment]): List of predefined segments.
            dp(float): Distance between individual points on the path.
        """
        # Variables
        self.segments = segment_list
        self.dp = dp

        # Private variables
        self._discrete_path = None
        self._xmin = None
        self._xmax = None
        self._ymin = None
        self._ymax = None

    @property
    def dpath(self):
        if self._discrete_path is None:
            self.compute_discrete_path()
        return self._discrete_path

    @property
    def xmin(self):
        if self._xmin is None:
            self.compute_bounding_box()
        return self._xmin

    @property
    def xmax(self):
        if self._xmax is None:
            self.compute_bounding_box()
        return self._xmax

    @property
    def ymin(self):
        if self._ymin is None:
            self.compute_bounding_box()
        return self._ymin

    @property
    def ymax(self):
        if self._ymax is None:
            self.compute_bounding_box()
        return self._ymax

    @property
    def centre(self):
        return [(self.xmin+self.xmax)/2, (self.ymin+self.ymax)/2]

    def get_index(self, index):
        """
        Checks that index is in range of the path length.
        If not, it wraps around.
        Args:
            index(int): Index of point on path

        Returns:
            int index
        """
        path_len = len(self.dpath)
        if 0 <= index < path_len:
            return index
        elif index >= path_len:
            index -= path_len
            return index

    def get_point(self, index):
        """
        Returns coordinates of the point at index.
        If index is larger than path length, it wraps around.
        Args:
            index(int): Index of point on path

        Returns:
            list[x,y]: point coordinates
        """
        i = self.get_index(index)
        return self.dpath[i]

    def get_closest(self, point):
        """
        Returns the index of point on path closest to the input.
        Args:
            point(list[x,y]): Point for which we wish to find closest.

        Returns:
            int: Index of closest point on path.
        """
        distance = (self.dpath[:, 1] - point[1]) ** 2 + (self.dpath[:, 0] - point[0]) ** 2
        i = np.where(distance == distance.min())
        return i[0][0]

    def get_point_at_distance(self, point, l, reverse=False):
        """
        Returns the index of point on path at distance l from the input point.
        If no point at distance is found, closest point is returned.
        Args:
            point(list[x,y]): Input point.
            l(float): Distance from input point.
            reverse(bool): If True, the found point direction will be reversed.

        Returns:
            int: Index of point on path at distance l from the input point.
        """
        # Get closest point
        closest_i = self.get_closest(point)

        # Get distances
        distance = (self.dpath[:, 1] - point[1]) ** 2 + (self.dpath[:, 0] - point[0]) ** 2

        # Return closest if out of radius
        if distance.min() > l**2:
            return closest_i

        # Get points closest to desired distances
        tol = 4
        i_lst = np.where(abs(distance - l**2) < (tol*self.dp)**2)

        # Loop until there are only 2 points close to distance l
        loop = 0
        while len(i_lst[0]) != 2:
            #print(i_lst)
            if loop > 50:
                return closest_i
            i_lst = np.where(abs(distance - l ** 2) < (tol * self.dp) ** 2)
            if len(i_lst[0]) < 2:
                tol += 0.03
            else:
                tol -= 0.03
            loop += 1
        i1 = i_lst[0][0]
        i2 = i_lst[0][1]

        # Get the right point
        path_len = len(self.dpath)
        if i2 > i1:
            di = i2 - i1
            if di < path_len-di:
                if reverse:
                    return i1
                return i2
            else:
                if reverse:
                    return i2
                return i1
        else:
            di = i1 - i2
            if di < path_len - di:
                if reverse:
                    return i2
                return i1
            else:
                if reverse:
                    return i1
                return i2

    def reposition(self, new_centre):
        """
        Repositions path, so its centre lies on the new_centre.
        Args:
            new_centre(list[x,y]): Desired centre of the path.
        """
        # Get translations
        dx = new_centre[0] - self.centre[0]
        dy = new_centre[1] - self.centre[1]

        # Move the path
        self._discrete_path[:, 0] += dx
        self._discrete_path[:, 1] += dy

        # Compute new bounding box
        self.compute_bounding_box()

    def compute_bounding_box(self):
        """
        Computes minimum and maximum x and y values for the path.
        """
        mins = np.amin(self.dpath, axis=0)
        maxes = np.amax(self.dpath, axis=0)
        self._xmin = mins[0]
        self._xmax = maxes[0]
        self._ymin = mins[1]
        self._ymax = maxes[1]

    def compute_discrete_path(self):
        """
        Computes discrete path points from its internal segments.
        Returns:
            list[x,y]: path points
        """
        # Insert all segments into path
        self._discrete_path = []
        for segment in self.segments:
            # Discretize segment
            points = segment.discretize(self.dp)
            # Append all points of the segment except the last one
            self._discrete_path.extend(points[:-1])
        # Insert last endpoint into path
        self._discrete_path.append(self.segments[-1].end_point)

        # Convert it to np.array
        self._discrete_path = np.array(self.dpath)

        # Get the bounding box
        self.compute_bounding_box()

        return self._discrete_path

    def plot(self):
        """
        Plots current path.
        """
        parr = self.dpath.T
        plt.plot(parr[0], parr[1], '*')
        plt.axis('equal')
        plt.grid()
        plt.show()
