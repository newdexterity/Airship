

class Segment(object):

    def __init__(self, start_point, end_point):
        """
        Initialises line segment.
        Args:
            start_point[x,y]: Starting point of the line.
            end_point[x,y]: Ending point of the line.
        """
        self.start_point = start_point
        self.end_point = end_point

    def discretize(self, segment_length):
        """
        Chops up the segment into portions of the same length.
        The segment length gets rounded to the closest value that allows same-length segments.
        Args:
            segment_length: Distance between individual segment points.

        Returns:
            list: segment points
        """
        pass
