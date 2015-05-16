import numpy as np

from geometry_msgs.msg import Pose
import nav_test

def test_trip_length():
    p0 = Pose()
    p1 = Pose()
    p1.position.x = 1.0
    p1.position.y = 1.0
    breadcrumbs = []
    breadcrumbs.append(p0)
    breadcrumbs.append(p1)
    l = nav_test.trip_length(breadcrumbs)
    assert l > 0
    assert np.isclose([l], [np.sqrt(2)])[0]

    p2 = Pose()
    p2.position.x = 2.0
    p2.position.y = 1.0
    breadcrumbs.append(p2)
    l = nav_test.trip_length(breadcrumbs)
    assert l > 0
    assert np.isclose([l], [np.sqrt(2) + 1.0])[0]
