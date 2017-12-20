#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import time
import yaml
import threading
import math
import numpy as np

LARGE_NUMBER = 2e32

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.image_lock = threading.RLock()


        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)


        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.trafficLightState = rospy.Publisher('/dbg/traffic_light_state', Image, queue_size = 1)

        self.switcher = {
                TrafficLight.RED: "RED",
                TrafficLight.YELLOW: "YELLOW",
                TrafficLight.GREEN: "GREEN",
            }

        self.colors = {"RED": (255, 0, 0), "YELLOW": (255, 255, 0),  "GREEN": (0, 255, 0) }

        rospy.logwarn('light detector')
        self.light_detector = rospy.get_param('~light_detector', False)
        rospy.logwarn('light detector')
        self.debug_window = rospy.get_param('~debug_window', False)

        rospy.spin()


    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        rospy.loginfo('Waypoints Received')

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = True
        self.camera_image = msg

        light_wp, state = self.process_traffic_lights()
        light_class = self.switcher.get(state, "UNKNOWN")


        rospy.logwarn('light {} light_wp {} state {}'.format(light_class, light_wp, Int32(state)))

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= rospy.get_param('~state_count_threshold', 3):
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        # if way points are empty
        if self.waypoints is None:
            return None

        # transform waypoints to array
        waypoints_array = np.asarray(
            [(w.pose.pose.position.x, w.pose.pose.position.y) for w in self.waypoints])

        position_array = np.asarray([pose.position.x, pose.position.y])
        #calculate euclidian distance
        distance = np.sum(np.sqrt((waypoints_array - position_array) ** 2), axis=1)
        # get closest
        index = np.argmin(distance)

        return index

    def get_distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def get_stop_line_waypoint(self, stop_line_positions):

        closest_idx = -1
        min_dist = LARGE_NUMBER
        for pos in stop_line_positions:
            stop_line = PoseStamped()
            stop_line.pose.position.x = float(pos[0])
            stop_line.pose.position.y = float(pos[1])

            dist = self.get_distance(stop_line.pose.position, self.pose.pose.position)
            if dist < min_dist:
                closest_idx = self.get_closest_waypoint(stop_line.pose)
                min_dist = dist

        return closest_idx

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light = None
        light_wp = None
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        config_string = rospy.get_param("/traffic_light_config")
        stop_line_positions = yaml.load(config_string)['stop_line_positions']
        #self.stop_line_wps = self.get_stop_line_waypoints(stop_line_positions)

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if (self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        # TODO find the closest visible traffic light (if one exists)
        if self.light_detector:
            time_start = time.time()
            state, score = self.light_classifier.get_classification(cv_image)

            #rospy.logwarn('light_wp_idx: {0}'.format(light_wp))
            light_wp = self.get_stop_line_waypoint(stop_line_positions)
            #rospy.logwarn('light_wp_idx: {0}'.format(light_wp))

            if self.debug_window:
                if state == TrafficLight.UNKNOWN:
                    self.trafficLightState.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(np.zeros((600, 800), np.uint8), cv2.COLOR_GRAY2RGB), "rgb8"))
                else:
                    light_class = self.switcher.get(state, "UNKNOWN")
                    cv2.putText(cv_image, "%s %f" % (light_class, score),
                                (10, 550),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1,
                                self.colors.get(light_class),
                                2)

                    cv2.rectangle(cv_image, (0, 0), (600, 800), state, 1)
                    self.trafficLightState.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))

            time_end = time.time()

            #rospy.loginfo('Traffic lights detection (ms) {} '.format((time_end - time_start) * 1000))

            return light_wp, state

        self.waypoints = None
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
