# A traced local midline: sequence of midpoints walking forward from the car
# along the centerline between the two visible track walls. The first point is
# the trace seed (typically the car's reference point at the origin).
# z is unused.

std_msgs/Header header
geometry_msgs/Point[] points
