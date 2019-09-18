import numpy as np
import math
from . import Segment


class Line(Segment.Segment):

    def __init__(self, start_point, end_point):
        """
        Initialises line segment.
        Args:
            start_point[x,y]: Starting point of the line.
            end_point[x,y]: Ending point of the line.
        """
        # Call to parent
        super(Line, self).__init__(start_point=start_point, end_point=end_point)

    @property
    def length(self):
        return math.sqrt((self.end_point[0] - self.start_point[0])**2 + (self.end_point[1] - self.start_point[1])**2 )

    def discretize(self, segment_length):
        # Get number of segments
        n_seg = self.length / segment_length
        # Round number of segments to nearest int
        n_seg = int(round(n_seg))
        # Get x and y coordinates
        x = np.linspace(self.start_point[0], self.end_point[0], n_seg, endpoint=True)
        y = np.linspace(self.start_point[1], self.end_point[1], n_seg, endpoint=True)
        # Make a list of them
        d = np.array([x, y]).T.tolist()
        return d

