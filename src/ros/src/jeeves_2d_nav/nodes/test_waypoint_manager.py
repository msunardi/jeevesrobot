"""Tests for the Waypoint class, and for methods of the waypoint_manager
module.
"""
import py
import yaml

import waypoint_manager as mgr


def test_waypoint(tmpdir):
    # write a waypoint file
    file_path = str(tmpdir.join('waypoints.yaml'))
    d = {'name': 'default', 'x': 0.0, 'y': 0.0, 'theta': 0.0, 'enabled': True}
    waypoints = []
    waypoints.append(mgr.Waypoint(d))
    waypoints.append(mgr.Waypoint(d))

    s = yaml.dump([wp.as_dict() for wp in waypoints], default_flow_style=False)
    with open(file_path, 'w') as f:
        f.write(s)
    print s
    # nothing to check here; just want to see what comes out
    
    # dict interface tests
    assert(waypoints[0]['x'] == 0.0)
    assert(waypoints[0]['y'] == 0.0)
    assert(waypoints[0]['theta'] == 0.0)
    assert(waypoints[0]['enabled'] == True)
    