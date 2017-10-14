#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
from std_msgs.msg import Int32, Float32

import math
import tf
from copy import deepcopy
from scipy.interpolate import CubicSpline

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
STOP_DIST = 2.0  # (in meters) Distance from closest traffic light to decide whether to top or go through intersection
ACCEL = 1.0  # Velocity increment (m/s) to be applied at each waypoint


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.base_waypoints = None
        self.final_waypoints = None
        self.current_pose = None
        self.next_waypoint_index = None
        self.light_wp = None
        self.max_speed = None
        self.slow_dist = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        self.wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.loop()

    def loop(self):
        rate = rospy.Rate(5)

        # While ROS is running
        while not rospy.is_shutdown():
            is_initialized = self.base_waypoints and self.current_pose and self.light_wp

            # If we already initialized
            if is_initialized:
                wpstart_idx = self.closest_waypoint(self.current_pose.position)
                # If this waypoint is behind the current pose then update to next waypoint
                self.next_waypoint_index = self.ahead_waypoint(wpstart_idx)
                # Set the waypoints' speeds and publish results
                self.publish()
            rate.sleep()

    def publish(self):

        final_waypoints_msg = Lane()
        final_waypoints_msg.header.stamp = rospy.Time.now()

        self.set_final_waypoints()
        self.set_final_waypoints_speed()
        final_waypoints_msg.waypoints = self.final_waypoints

        self.final_waypoints_pub.publish(final_waypoints_msg)

    def set_final_waypoints(self):
        # Grab the next LOOKAHEAD_WPS waypoints from the base waypoints.
        self.final_waypoints = deepcopy(
            self.base_waypoints[self.next_waypoint_index: self.next_waypoint_index + LOOKAHEAD_WPS])

        # If wraparound occurs, handle it by grabbing waypoints from beginning of base waypoints.
        rem_points = LOOKAHEAD_WPS - len(self.final_waypoints)
        if rem_points > 0:
            self.final_waypoints = self.final_waypoints + deepcopy(self.base_waypoints[0:rem_points])

    def set_final_waypoints_speed(self):
        # Get the distance between the next waypoint and the traffic light.
        dist = self.distance(self.base_waypoints, self.next_waypoint_index, abs(self.light_wp))

        no_red_light_detected = self.light_wp < 0
        # IF no RED light,
        if no_red_light_detected:
            # rospy.logwarn("Upcoming TrafficLight %d" % -self.light_wp)
            speed = self.current_velocity
            target_speed = self.max_speed
            # rospy.logwarn("No traffic light, so accelerate to top speed! ")

            Creep_Enabled = False
            if Creep_Enabled and dist < self.slow_dist:
                target_speed = self.max_speed / 2.0
            for wp in self.final_waypoints:
                # if speed > target_speed:
                #     speed = min(target_speed, speed + ACCEL)
                # else:
                #     speed = max(target_speed, speed - ACCEL)
                speed = target_speed
                self.set_waypoint_velocity(wp, speed)
            return
        # Report message of upcoming traffic light.
        # rospy.logwarn("Next wp: %s, Next TL wp: %s, distance: %s", self.next_waypoint_index, self.light_wp, dist)
        if dist <= self.slow_dist:
            speed = max(self.current_velocity, max(dist * 0.2, 0.3))
            # speed = self.current_velocity
            if dist > STOP_DIST:
                decel = speed / (dist - STOP_DIST)
                rospy.logwarn("*** Decelerating! My next wp: %s, is close to Next TL wp: %s, So decelerate to with : %f m/s", self.next_waypoint_index, self.light_wp, decel)
            else:
                speed = 0
                decel = 0
                rospy.logwarn("!!! You are into intersection while RED light on! Vehicle Stopped")
            for wp in self.final_waypoints:
                speed = max(0, speed - decel)
                self.set_waypoint_velocity(wp, speed)
        else:
            speed = self.current_velocity
            if speed < self.max_speed:
                rospy.logwarn("~~~ Traffic light far away, so accelerate to top speed! ")
                for wp in self.final_waypoints:
                    # if speed > self.max_speed:
                    #     speed = min(self.max_speed, speed + ACCEL)
                    # else:
                    #     speed = max(self.max_speed, speed - ACCEL)
                    speed = self.max_speed
                    self.set_waypoint_velocity(wp, speed)

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose

    # if self.waypoints is not None:
    #     self.final_waypoints_pub.publish(get_waypoints_infront(self.current_pose, self.waypoints))

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints.waypoints
        if self.base_waypoints:
            self.wp_sub.unregister()
            self.wp_sub = None
            # self.max_speed = self.get_waypoint_velocity(self.base_waypoints[0])  # get velocity from waypoint loader
            self.max_speed = 12  # get velocity from waypoint loader
            # self.slow_dist = self.max_speed * 5
            self.slow_dist = max(self.get_waypoint_velocity(self.base_waypoints[0]) * 5, self.max_speed * 3)

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.light_wp = msg.data
        # rospy.logwarn("self.light_wp = %s", self.light_wp)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        # self.obstacle_waypoint = msg.data
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist, wp3 = 0.0, -1
        # In case of wraparound
        if wp2 < wp1:
            wp3, wp2 = wp2, len(waypoints) - 1
        for i in xrange(wp1, wp2):
            dist += self.dist_bw_wp(waypoints[i].pose.pose.position, waypoints[i + 1].pose.pose.position)
        for i in xrange(-1, wp3):
            dist += self.dist_bw_wp(waypoints[i].pose.pose.position, waypoints[i + 1].pose.pose.position)
        return dist

    def dist_bw_wp(self, p, q):
        dist = math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2)
        return dist

    def closest_waypoint(self, position):
        return min(xrange(len(self.base_waypoints)),
                   key=lambda p: self.dist_bw_wp(position, self.base_waypoints[p].pose.pose.position))

    def get_next_waypoint(self, pose, waypoints):
        closest_wp = self.get_closest_waypoint(pose, waypoints)
        wp_x = waypoints[closest_wp].pose.pose.position.x
        wp_y = waypoints[closest_wp].pose.pose.position.y
        heading = math.atan2((wp_y - pose.pose.position.y), (wp_x - pose.pose.position.x))
        x = pose.pose.orientation.x
        y = pose.pose.orientation.y
        z = pose.pose.orientation.z
        w = pose.pose.orientation.w
        euler_angles_xyz = tf.transformations.euler_from_quaternion([x, y, z, w])
        theta = euler_angles_xyz[-1]
        angle = math.fabs(theta - heading)
        if angle > math.pi / 4.0:
            closest_wp += 1

        return closest_wp

    def get_euler_yaw(self):
        quaternion = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def ahead_waypoint(self, wp_idx):
        ahead_idx = wp_idx

        # Get coordinates of the waypoint at the specified index.
        map_wp_x = self.base_waypoints[wp_idx].pose.pose.position.x
        map_wp_y = self.base_waypoints[wp_idx].pose.pose.position.y

        # Get current position
        x, y = self.current_pose.position.x, self.current_pose.position.y

        # Get yaw angle
        yaw = self.get_euler_yaw()

        # Compute expression to determine if closest waypoint is behind us or not.
        localize_x = (map_wp_x - x) * math.cos(yaw) + (map_wp_y - y) * math.sin(yaw)

        # If the waypoint is behind us, then increment the waypoint index.
        if localize_x < 0.0:
            ahead_idx = ahead_idx + 1

        # Set the internal property
        self.ahead_waypoint_index = ahead_idx

        # And return the index as a return argument.
        return ahead_idx


    #
    # # yaw of vector from p to q
    # def angle(p, q):
    #     return math.atan2(q.y - p.y, q.x - p.x)

    # find the nearest way point to the current pose
    # Note that this point can be in front or behind the current pose
    # def get_nearest_wp(current_pose, waypoints):
    #     nearest_wp_id = -1
    #     nearest_wp_dist_sqr = 1e100
    #
    #     for i in range(len(waypoints.waypoints)):
    #         wp_dist_sqr = dist_sqr(current_pose.pose.position, waypoints.waypoints[i].pose.pose.position)
    #         if wp_dist_sqr < nearest_wp_dist_sqr:
    #             nearest_wp_dist_sqr = wp_dist_sqr
    #             nearest_wp_id = i
    #
    #     return nearest_wp_id
    #
    # # Find several way points in front of the current pose
    # def get_waypoints_infront(current_pose, waypoints):
    #     # the id of nearest way point
    #     nearest_wp_id = get_nearest_wp(current_pose, waypoints)
    #
    #     # the nearst way point itself
    #     nearest_wp = waypoints.waypoints[nearest_wp_id].pose
    #
    #     # We need to check whether this way point is in front or behind the current pose
    #
    #     # road orientation (in 2d)
    #     road_orientation = nearest_wp.pose.orientation
    #     road_quaternion = (road_orientation.x, road_orientation.y, road_orientation.z, road_orientation.w)
    #     road_euler = tf.transformations.euler_from_quaternion(road_quaternion)
    #     road_yaw = road_euler[-1]  # roll = euler[0], pitch = euler[1], yaw = euler[2]
    #
    #     # yaw of vector connecting way point and current pose
    #     car_to_wp_yaw = angle(current_pose.pose.position, nearest_wp.pose.position)
    #     yaw_diff = abs(road_yaw - car_to_wp_yaw)
    #     if yaw_diff > math.pi:
    #         yaw_diff = 2 * math.pi - yaw_diff
    #
    #     # obtuse angle means car is moving away from the way point, thus use the next way point
    #     if yaw_diff > math.pi / 2.0:
    #         nearest_wp_id += 1
    #
    #     j = nearest_wp_id
    #
    #     lookahead = []
    #     for i in range(LOOKAHEAD_WPS):
    #         lookahead.append(waypoints.waypoints[j % len(waypoints.waypoints)])
    #         j += 1
    #
    #     final_waypoints = Lane()
    #     final_waypoints.header.frame_id = '/world'
    #     final_waypoints.header.stamp = rospy.Time.now()
    #     final_waypoints.waypoints = lookahead
    #
    #     # rospy.logwarn('Car is at (%s,%s), car_yaw=%s, road_yaw=%s, yaw_diff=%s', current_pose.pose.position.x, current_pose.pose.position.y, car_to_wp_yaw, road_yaw, yaw_diff)
    #     # wp_coord = [(wp.pose.pose.position.x, wp.pose.pose.position.y) for wp in final_waypoints.waypoints]
    #     # wp_coord_str = '\n'.join(['({},{})'.format(x,y) for x,y in wp_coord[:20]])
    #     # rospy.logwarn('Way points ahead:\n%s', wp_coord_str)
    #
    #     return final_waypoints
    # def get_final_waypoints(self, waypoints, start_up, end_up):
    #     final_waypoints = []
    #     for i in range(start_up, end_up):
    #         index = i%len(waypoints)
    #         wp = Waypoint()
    #         wp.pose.pose.position.x = waypoints[index].pose.pose.position.x
    #         wp.pose.pose.position.y = waypoints[index].pose.pose.position.y
    #         wp.pose.pose.position.z = waypoints[index].pose.pose.position.z
    #         wp.pose.pose.orientation = waypoints[index].pose.pose.orientation
    #         if self.braking:
    #             # Slowly creep up to light if we have stopped short
    #             dist = self.distance(wp.pose.pose.position, waypoints[end_wp].pose.pose.position)
    #             if dist > STOP_BUFFER and self.current_velocity < 1.0:
    #                 wp.twist.twist.linear.x = 2.0
    #             elif dist < STOP_BUFFER and self.current_velocity < 1.0:
    #                 wp.twist.twist.linear.x = 0.0
    #             else:
    #                 wp.twist.twist.linear.x = min(self.current_velocity, waypoints[index].twist.twist.linear.x)
    #         else:
    #             wp.twist.twist.linear.x = waypoints[index].twist.twist.linear.x
    #         final_waypoints.append(wp)
    #     if self.braking:
    #         # Find the traffic_wp index in final_waypoints to pass to decelerate
    #         tl_wp = len(final_waypoints)
    #
    #         # If we are braking set all waypoints passed traffic_wp within LOOKAHEAD_WPS to 0.0
    #         for i in range(end_wp, start_wp + LOOKAHEAD_WPS):
    #             index = i % len(waypoints)
    #             wp = Waypoint()
    #             wp.pose.pose.position.x  = waypoints[index].pose.pose.position.x
    #             wp.pose.pose.position.y  = waypoints[index].pose.pose.position.y
    #             wp.pose.pose.position.z  = waypoints[index].pose.pose.position.z
    #             wp.pose.pose.orientation = waypoints[index].pose.pose.orientation
    #             wp.twist.twist.linear.x  = 0.0
    #             final_waypoints.append(wp)
    #         final_waypoints = self.decelerate(final_waypoints, tl_wp)
    #
    #     return final_waypoints

    # def decelerate(self, waypoints, tl_wp):
    #     last = waypoints[tl_wp]
    #     last.twist.twist.linear.x = 0.0
    #     for wp in waypoints[:tl_wp][::-1]:
    #         dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
    #         dist = max(0.0, dist - STOP_BUFFER)
    #         vel  = math.sqrt(2 * self.decel * dist)
    #         if vel < 1.0:
    #             vel = 0.0
    #         wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
    #     return waypoints

    # def trafficlight_cb(self):
    #     # self.lights = msg.lights
    #     pass

    # def distance(self, waypoints, wp1, wp2):
    #     dist = 0
    #     dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    #     for i in range(wp1, wp2+1):
    #         dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
    #         wp1 = i
    #     return dist
    # def distance(self, p1, p2):
    #     x = p1.x - p2.x
    #     y = p1.y - p2.y
    #     z = p1.z - p2.z
    #     return math.sqrt(x*x + y*y + z*z)

    # def get_closest_waypoint(self, pose, waypoints):
    #     closest_dist = float('inf')
    #     closest_wp = 0
    #     for i in range(len(waypoints)):
    #         dist = self.distance(pose.pose.position, waypoints[i].pose.pose.position)
    #         if dist < closest_dist:
    #             closest_dist = dist
    #             closest_wp = i
    #
    #     return closest_wp


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
