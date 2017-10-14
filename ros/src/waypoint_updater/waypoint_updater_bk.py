#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

# Square distance between 2 points
def dist_sqr(p, q):
    return (p.x-q.x)**2 + (p.y-q.y)**2

# yaw of vector from p to q
def angle(p, q):
    return math.atan2(q.y-p.y, q.x-p.x)


# find the nearest way point to the current pose
# Note that this point can be in front or behind the current pose
def get_nearest_wp(current_pose, waypoints):
    nearest_wp_id = -1
    nearest_wp_dist_sqr = 1e100

    for i in range(len(waypoints.waypoints)):
        wp_dist_sqr = dist_sqr(current_pose.pose.position, waypoints.waypoints[i].pose.pose.position)
        if wp_dist_sqr < nearest_wp_dist_sqr:
            nearest_wp_dist_sqr = wp_dist_sqr
            nearest_wp_id = i

    return nearest_wp_id

# Find several way points in front of the current pose
def get_waypoints_infront(current_pose, waypoints):
    # the id of nearest way point
    nearest_wp_id = get_nearest_wp(current_pose, waypoints)

    # the nearst way point itself
    nearest_wp = waypoints.waypoints[nearest_wp_id].pose

    # We need to check whether this way point is in front or behind the current pose

    # road orientation (in 2d)
    road_orientation = nearest_wp.pose.orientation
    road_quaternion = (road_orientation.x, road_orientation.y, road_orientation.z, road_orientation.w)
    road_euler = tf.transformations.euler_from_quaternion(road_quaternion)
    road_yaw = road_euler[-1]

    # yaw of vector connecting way point and current pose
    car_to_wp_yaw = angle(current_pose.pose.position, nearest_wp.pose.position)
    yaw_diff = abs(road_yaw - car_to_wp_yaw)
    if yaw_diff > math.pi:
        yaw_diff = 2*math.pi - yaw_diff

    # obtuse angle means car is moving away from the way point, thus use the next way point
    if yaw_diff > math.pi/2.0:
        nearest_wp_id += 1

    j = nearest_wp_id

    lookahead = []
    for i in range(LOOKAHEAD_WPS):
        lookahead.append(waypoints.waypoints[j %  len(waypoints.waypoints)])
        j += 1

    final_waypoints = Lane()
    final_waypoints.header.frame_id = '/world'
    final_waypoints.header.stamp = rospy.Time.now()
    final_waypoints.waypoints = lookahead

    # rospy.logwarn('Car is at (%s,%s), car_yaw=%s, road_yaw=%s, yaw_diff=%s', current_pose.pose.position.x, current_pose.pose.position.y, car_to_wp_yaw, road_yaw, yaw_diff)
    # wp_coord = [(wp.pose.pose.position.x, wp.pose.pose.position.y) for wp in final_waypoints.waypoints]
    # wp_coord_str = '\n'.join(['({},{})'.format(x,y) for x,y in wp_coord[:20]])
    # rospy.logwarn('Way points ahead:\n%s', wp_coord_str)

    return final_waypoints



class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.waypoints = None
        self.current_pose = None

        rospy.spin()


    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg

        if self.waypoints is not None:
            self.final_waypoints_pub.publish(get_waypoints_infront(self.current_pose, self.waypoints))

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
