from Path2D import Line, Arc, Path
import matplotlib.pyplot as plt
import numpy as np

a1 = Arc.Arc([0, 1], [1, 2], 360)

#l1 = Line.Line(a1.start_point, a1.end_point)

path = Path.Path([a1], 0.1)

path.reposition([0, 0])

point = [-1.5, -1.5]
dpoint = path.get_point_at_distance(point, 1, False)

plt.plot(point[0], point[1], 'ro', markersize=10)
plt.plot(path.dpath[dpoint][0], path.dpath[dpoint][1], 'go', markersize=10)
plt.plot(path.dpath[0][0], path.dpath[0][1], 'bo', markersize=10)
path.plot()
exit()
closest = path.get_closest(point)
print(closest)

print(path.dpath[closest])

plt.plot(point[0], point[1], 'ro', markersize=10)
plt.plot(path.dpath[closest][0], path.dpath[closest][1], 'go', markersize=10)
path.plot()

p = path.get_point(1)
print(len(path.dpath), p)
p = path.get_point(78)
print(len(path.dpath), p)



