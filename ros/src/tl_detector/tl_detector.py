#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np

STATE_COUNT_THRESHOLD = 3
# Maximum traffic light distance to care about.
MAX_LIGHT_DISTANCE = 120.

class TLDetector(object):
    def __init__(self, enable_location_zoom=False, enable_test_light_state=True):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.num_waypoints = 0
        self.camera_image = None
        self.lights = None
        # Used in testing - latest lights information.
        self.latest_lights = None
        # Enable zooming in on the traffic light location on the image. This
        # requires a calibrated camera with precise focal length etc.
        self.enable_location_zoom = enable_location_zoom

        # Enable using the light state information from the /vehicle/traffic_lights
        # topic.
        #
        # NOTES:
        # 1. Set this to False to use the trained classifier.
        # 2. Set this to True to use the reported traffic light state without the classifier.
        self.enable_test_light_state = enable_test_light_state

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber(
            '/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        self.traffic_lights_sub = rospy.Subscriber('/vehicle/traffic_lights',
                                                   TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.lights_waypoint_lookup = None
        self.lights_stopline_waypoint_lookup = None
        self.initialized = False

        self.distance = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        self.direction = lambda a, b: math.atan2(a.y-b.y, a.x-b.x)
        self.uid = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        if waypoints is not None:
            self.waypoints = waypoints
            self.num_waypoints = len(self.waypoints.waypoints)
            # constant information
            self.base_waypoints_sub.unregister()
            self.init_lights_waypoint_lookup_if_ready()
        
    def traffic_cb(self, msg):
        if len(msg.lights) > 0:
            self.latest_lights = msg.lights
            if self.lights is None:
                self.lights = msg.lights
            if not self.initialized:
                rospy.loginfo('msg.lights length {}'.format(len(msg.lights)))            
                self.init_lights_waypoint_lookup_if_ready()
            if not self.enable_test_light_state:
                # constant information that doens't change.
                self.traffic_lights_sub.unregister()
            else:
                if not self.initialized:
                    rospy.loginfo('Using reported traffic light state for testing.')

    def init_lights_waypoint_lookup_if_ready(self):
        """Initialize for quick lights waypoint lookup.
        """
        if self.waypoints is not None and self.lights is not None:
            rospy.loginfo('waypoints length {}'.format(self.num_waypoints))
            rospy.loginfo('lights length {}'.format(len(self.lights)))
            self.lights_waypoint_lookup = []
            self.lights_stopline_waypoint_lookup = []
            stop_line_positions = self.config['stop_line_positions']
            stop_line_waypoints = []
            stop_line_points = []
            for stop_line_position in stop_line_positions:
                stop_line_point = Point()
                stop_line_point.x = stop_line_position[0]
                stop_line_point.y = stop_line_position[1]
                stop_line_points.append(stop_line_point)
                min_dist = 1e9
                min_i = 0
                for wi, waypoint in enumerate(self.waypoints.waypoints):
                    dist = self.distance(stop_line_point, waypoint.pose.pose.position)
                    if dist < min_dist:
                        min_i = wi
                        min_dist = dist
                stop_line_waypoints.append(min_i)
            for li, light in enumerate(self.lights):
                min_dist = 1e9
                min_i = 0
                for wi, waypoint in enumerate(self.waypoints.waypoints):
                    dist = self.distance(
                        light.pose.pose.position, waypoint.pose.pose.position)
                    if dist < min_dist:
                        min_i = wi
                        min_dist = dist
                self.lights_waypoint_lookup.append(min_i)
                min_dist = 1e9
                min_i = 0
                for si, stop_line_point in enumerate(stop_line_points):
                    dist = self.distance(
                        light.pose.pose.position, stop_line_point)
                    if dist < min_dist:
                        min_i = si
                        min_dist = dist
                self.lights_stopline_waypoint_lookup.append(stop_line_waypoints[min_i])
                    
            self.initialized = True
            rospy.loginfo('lights_waypoint_lookup length {}: {}'.format(
                len(self.lights_waypoint_lookup), self.lights_waypoint_lookup))
            rospy.loginfo('lights_stopline_waypoint_lookup length {}: {}'.format(
                len(self.lights_stopline_waypoint_lookup),
                self.lights_stopline_waypoint_lookup))
                          
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_yaw(self, pose):
        """Gets the yaw angle of the car pose.
        """
        euler = tf.transformations.euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w])
        # roll pitch yaw
        return euler[2]

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        if not self.waypoints or not pose:
            return -1

        closest = 0
        min_distance = 1e9
        current_position = pose.position
        idx = 0
        # TODO xingjin2017: do some sort of sorted search.
        for waypoint in self.waypoints.waypoints:
            way_position = waypoint.pose.pose.position
            distance = self.distance(way_position, current_position)
            if distance < min_distance:
                closest = idx
                min_distance = closest
            idx += 1
        closest_position = self.waypoints.waypoints[closest].pose.pose.position
        direction = self.direction(closest_position, current_position)
        # The waypoint is behind the car.
        if np.abs(self.get_yaw(pose) - direction) > np.pi/4.0:
            closest += 1
            if closest >= len(self.waypoints.waypoints):
                closest = 0
        return closest

    def project_to_image_plane(self, point_in_world, offset_y=0, offset_z=0, debug=False):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        if trans is None or rot is None:
            return -1, -1

        # Camera center on the image
        cx = image_width/2
        cy = image_height/2

        px = point_in_world.x
        py = point_in_world.y
        pz = point_in_world.z

        rpy = tf.transformations.euler_from_quaternion(rot)
        yaw = rpy[2]
        point_in_car = (px * math.cos(yaw) - py * math.sin(yaw),
                        px * math.sin(yaw) + py * math.cos(yaw),
                        pz)
        point_in_car = [sum(x) for x in zip(point_in_car, trans)]

        point_in_car[1] = point_in_car[1] + offset_y
        point_in_car[2] = point_in_car[2] + offset_z

        # To get around unavailable/invalid camera fx/fy in simulator:
        # https://discussions.udacity.com/t/focal-length-wrong/358568/22
        if fx < 10:
            fx = 2574
            fy = 2744
            point_in_car[2] -= 1.0
            cx = image_width/2 - 30
            cy = image_height + 50
        x = int(-point_in_car[1] * fx / point_in_car[0] + cx)
        y = int(-point_in_car[2] * fy / point_in_car[0] + cy)

        return (x, y)   
        
    def get_closest_traffic_light(self, car_wp):
        """Identifies the closest traffic in front of the given position
        Returns:
            index to self.lights array

        """
        #TODO implement
        if not self.pose or not self.lights:
            return None

        closest = None
        min_distance = 1e9
        current_position = self.pose.pose.position
        for li, light in enumerate(self.lights):
            light_wp = self.lights_waypoint_lookup[li]
            light_position = light.pose.pose.position
            distance = self.distance(light_position, current_position)
            light_in_front = light_wp >= car_wp or \
                             car_wp > light_wp + 0.9 * self.num_waypoints 
            if light_in_front and distance < min_distance and \
               distance < MAX_LIGHT_DISTANCE:
                closest = li
                min_distance = closest
        return closest

    def get_closest_stop_line_waypoint(self, light_idx):
        """Gets the waypoint index for the stopline that's cloest to the light.
        
        Args:
           light_idx: index into self.lights array.

        Returns:
           The waypoint index that's cloest to the stopline that maps to the
              traffic light.
        """
        return self.lights_stopline_waypoint_lookup[light_idx]
    
    def get_test_light_state(self, light):
        """Gets the simulation reported traffic light state.

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.latest_lights is None:
            return TrafficLight.UNKNOWN

        match = None
        min_distance = 1e9
        for l in self.latest_lights:
            dist = self.distance(
                light.pose.pose.position, l.pose.pose.position)
            if dist < min_distance:
                min_distance = dist
                match = l
        if min_distance < 1e-2:
            return match.state
        else:
            return TrafficLight.UNKNOWN

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.latest_lights:
            
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        if self.enable_location_zoom:
            # Two pairs of points that mark the top left and bottom right of
            # cropping region - 3D world/car coordinates in meters (not image).
            crop_width = 1.0
            crop_height = 2.0
            x1, y1 = self.project_to_image_plane(light.pose.pose.position,
                                                 crop_width/2, crop_height/2)
            x2, y2 = self.project_to_image_plane(light.pose.pose.position,
                                                 -crop_width/2, -crop_height/2)
            image_h, image_w, _ = cv_image.shape
            rospy.loginfo('light top left ({}, {}), bottom right ({}, {})'
                          .format(x1, y1, x2, y2))
            x1 = max(0, min(x1, image_w-1))
            y1 = max(0, min(y1, image_h-1))
            x2 = max(0, min(x2, image_w-1))
            y2 = max(0, min(y2, image_w-1))
            if x2 - x1 < 20 or y2 - y1 < 20:
                return TrafficLight.UNKNOWN

            if debug:
                marked_img = cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                oname = '/home/xjin/tmp/lights/lights_{:05d}.png'.format(self.uid)
                w = abs(x1 - x2)
                h = abs(y1 - y2)
                rospy.loginfo('saving to {} with region {}x{}'.format(oname, w, h))
                self.uid += 1
                cv2.imwrite(oname, marked_img)

            #TODO use light location to zoom in on traffic light in image
            cropped = cv_image[y1:y2, x1:x2]
            cv_image = cropped

        #Get classification
        return self.light_classifier.get_classification(cv_image)
        #return self.light_classifier.get_classification_by_non_train(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # Not ready for business yet.
        if self.pose is None or not self.initialized:
            return -1, TrafficLight.UNKNOWN

        car_position = self.get_closest_waypoint(self.pose.pose)
        #TODO find the closest visible traffic light (if one exists)
        light_idx = self.get_closest_traffic_light(car_position)
            
        if light_idx is not None:
            light = self.lights[light_idx]
            light_wp = self.get_closest_stop_line_waypoint(light_idx)
            if self.enable_test_light_state:
                state = self.get_test_light_state(light)
            else:
                state = self.get_light_state(light)
            #rospy.loginfo('light_wp {} state {}'.format(light_wp, state))
            return light_wp, state

        # This has been deliberatedly commented out as it is being reused.
        #self.waypoints = None

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector(False, False)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
