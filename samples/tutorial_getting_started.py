#!/usr/bin/env python
import roslib; roslib.load_manifest('jrl_walkgen_bridge')
import rospy
from walk_msgs.GetPathClient \
    import Client
import walk_msgs.msg

if __name__ == "__main__":
    print "Calling getPath service..."
    try:
        client = Client()
        client.initial_left_foot_position.position.y = -0.19
        client.initial_right_foot_position.position.y = +0.19
        client.initial_center_of_mass_position.z = 0.8
        client.final_left_foot_position.position.y = -0.19
        client.final_right_foot_position.position.y = +0.19
        client.final_center_of_mass_position.z = 0.8
        client.start_with_left_foot = True
        client.footprints = []

        client.final_left_foot_position.position.x = 3 * 0.25
        client.final_right_foot_position.position.x = 3 * 0.25

        def makeFootprint(x, y):
            footprint = walk_msgs.msg.Footprint2d()
            footprint.beginTime.secs = 0
            footprint.beginTime.nsecs = 0
            footprint.duration.secs = 0.
            footprint.duration.nsecs = 1. * 1e9
            footprint.x = x
            footprint.y = y
            footprint.theta = 0.

            print("{0} {1}".format(footprint.x, footprint.y))
            return footprint


        client.footprints.append(makeFootprint(1 * 0.25, -0.19))
        client.footprints.append(makeFootprint(1 * 0.25, +0.19))

        client.footprints.append(makeFootprint(2 * 0.25, -0.19))
        client.footprints.append(makeFootprint(2 * 0.25, +0.19))

        client.footprints.append(makeFootprint(3 * 0.25, -0.19))
        client.footprints.append(makeFootprint(3 * 0.25, +0.19))

        if not not client():
            print("Service call succeed")
        else:
            print("Service call failed")

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
