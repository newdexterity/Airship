import numpy as np
import math
from . import Segment


class Arc(Segment.Segment):

    def __init__(self, start_point, centre, angle):
        """
        Initialises line segment.
        Args:
            start_point(list[x,y]): Starting point of the arc.
            centre(list[x,y]): Centre of the arc.
            angle(deg): Angle of the arc; defines length.
        """
        # Convert angle
        self.angle = angle * math.pi / 180

        # Check direction
        if angle < 0:
            self.angle = -self.angle
            self.angle_reverse = True
        else:
            self.angle_reverse = False

        # Get end point
        end_point = self.rotate_vector(origin=centre, end_point=start_point, angle=self.angle)

        # Call to parent
        super(Arc, self).__init__(start_point=start_point, end_point=end_point)

        # Save other params
        self.centre = centre
        self.radius = math.sqrt((self.start_point[0] - self.centre[0])**2 + (self.start_point[1] - self.centre[1])**2 )

    @property
    def length(self):
        return self.angle * self.radius

    def rotate_vector(self, origin, end_point, angle):
        """
        Rotates vector and returns coordinate of the rotated end point.
        Args:
            origin(list[x,y]): Starting point of the vector
            end_point(list[x,y]): Ending point of the vector.
            angle(RAD): Angle of the arc; defines length.

        Returns:
            list[x,y]: end point
        """
        # Define vectors
        OC = np.array(origin)
        OA = np.array(end_point)
        CA = OA - OC

        # Reverse direction if specified
        if not self.angle_reverse:
            angle = -angle

        # Compute CB by vector rotation
        cbx = CA[0]*math.cos(angle) + CA[1]*math.sin(angle)
        cby = -CA[0]*math.sin(angle) + CA[1]*math.cos(angle)
        CB = np.array([cbx, cby])

        # Get OB
        OB = OC + CB

        return OB

    def discretize(self, segment_length):
        # Get number of segments
        n_seg = self.length / segment_length
        # Round number of segments to nearest int
        n_seg = int(round(n_seg))
        # Get angle delta
        d_angle = self.angle / n_seg

        # Loop and compute points
        points = [self.start_point]
        for i in range(n_seg):
            points.append(self.rotate_vector(self.centre, points[-1], d_angle))

        return points

