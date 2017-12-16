#!/usr/bin/env python

import math
import rospy
import tf

from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from styx_msgs.msg import TrafficLightArray, TrafficLight
import yaml

import numpy as np
from lowpass import LowPassFilter
from bisect import bisect_right

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

LARGE_NUMBER = 2e32
LOOKAHEAD_WPS = 150 # Number of waypoints we will publish
MAXIMUM_ANGLE = math.pi / 4
MAXIMUM_DECELERATION = 0.5
STOPPING_DISTANCE = 5.0
KMH_TO_MPS = 0.44704

def waypoint_is_feasible(pose, waypoint):
    """
    Determine whether the angle from the current
    vehicle pose to the given waypoint is drivable
    """
    waypoint_x = waypoint.pose.pose.position.x
    waypoint_y = waypoint.pose.pose.position.y
    heading_to_waypoint = math.atan2((waypoint_y - pose.position.y), (waypoint_x - pose.position.x))
    _, _, yaw = tf.transformations.euler_from_quaternion((pose.orientation.x,
                                                          pose.orientation.y,
                                                          pose.orientation.z,
                                                          pose.orientation.w,))
    return abs(yaw - heading_to_waypoint) <= MAXIMUM_ANGLE

def set_waypoint_linear_velocity(waypoint, velocity):
    waypoint.twist.twist.linear.x = velocity

def get_distance(p1, p2):
    x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
    return math.sqrt(x*x + y*y + z*z)

def get_dist(p1, p2):
    x, y = p1[0] - p2[0], p1[1] - p2[1]
    return math.sqrt(x*x + y*y)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.loginfo('Waypoint Updater Initialized')

        top_speed = rospy.get_param("/waypoint_loader/velocity", None) # km/h
        assert top_speed is not None, "missing parameter?"

        self.target_velocity = top_speed * KMH_TO_MPS

        rospy.loginfo("Top velocity: {0} m/s".format(self.target_velocity))

        self.waypoints = None
        self.current_pose = None
        self.pose_frame_id = None
        self.next_waypoint_index = None
        self.traffic_light_index = -1
        self.stop_line_wps = []

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # setup stop line waypoints for debugging with light classifier
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_ground_truth_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def is_ready(self):
        return all((self.waypoints, self.current_pose))

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            if not self.is_ready():
                continue

            next_waypoint_index = self.get_next_waypoint_index()
            last_waypoint_index = next_waypoint_index + LOOKAHEAD_WPS
            lookahead_waypoints = deepcopy(self.waypoints[next_waypoint_index:last_waypoint_index])
            red_light_index = self.traffic_light_index - next_waypoint_index

            # if red light has been detected and is in range, decelerate
            if red_light_index > -1 and red_light_index < LOOKAHEAD_WPS:
                red_light_waypoint = lookahead_waypoints[red_light_index]

                for index, waypoint in enumerate(lookahead_waypoints):
                    if index > red_light_index:
                        velocity = 0.
                    else:
                        distance = get_distance(waypoint.pose.pose.position, red_light_waypoint.pose.pose.position)
                        distance = max(0., distance-STOPPING_DISTANCE)
                        velocity = math.sqrt(2 * MAXIMUM_DECELERATION * distance)
                        if velocity < 1.:
                            velocity = 0.
                    set_waypoint_linear_velocity(waypoint, velocity)
            else:
                for waypoint in lookahead_waypoints:
                    set_waypoint_linear_velocity(waypoint, self.target_velocity)

            self.publish(lookahead_waypoints)

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = self.pose_frame_id
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        self.pose_frame_id = msg.header.frame_id

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        rospy.loginfo('Waypoints Received')

        config_string = rospy.get_param("/traffic_light_config")
        stop_line_positions = yaml.load(config_string)['stop_line_positions']
        self.stop_line_wps = self.get_stop_line_waypoints(stop_line_positions)

    def traffic_cb(self, waypoint):
        self.traffic_light_index = waypoint
        rospy.logwarn("Receiving traffic light info!")

    def traffic_ground_truth_cb(self, traffic_light_array):
        if not self.stop_line_wps:
            return

        red_stop_line_wps = []
        for index, light in enumerate(traffic_light_array.lights):
            if light.state == TrafficLight.RED:
                red_stop_line_wps.append(self.stop_line_wps[index])

        new_traffic_light_index = self.closest_light(red_stop_line_wps)

        if new_traffic_light_index != self.traffic_light_index:
            rospy.logwarn("GT red light waypoint = {0}".format(new_traffic_light_index))
            if new_traffic_light_index >= 0:
                light_wp = self.waypoints[new_traffic_light_index]
                light_line_pos = light_wp.pose.pose.position
                rospy.logwarn("pos = ({0}, {1})".format(light_line_pos.x, light_line_pos.y))

        self.traffic_light_index = new_traffic_light_index

    def get_stop_line_waypoints(self, stop_line_positions):
        stop_line_wps = []

        for p in stop_line_positions:
            closest_idx = -1
            min_dist = float('infinity')
            for index, waypoint in enumerate(self.waypoints):
                pos = waypoint.pose.pose.position
                pos = (pos.x, pos.y)
                dist = get_dist(p, pos)

                if dist < min_dist:
                    closest_idx = index
                    min_dist = dist

            stop_line_wps.append(closest_idx)
        return stop_line_wps

    def closest_light(self, red_stop_line_wps):
        closest_wp = -1
        min_dist = LARGE_NUMBER

        for stop_line_wp in red_stop_line_wps:
            curr_wp = self.waypoints[stop_line_wp]
            dist = get_distance(self.current_pose.position, curr_wp.pose.pose.position)

            if dist < 30:
                return stop_line_wp

            if not waypoint_is_feasible(self.current_pose, curr_wp):
                continue

            if dist < min_dist:
                min_dist = dist
                closest_wp = stop_line_wp

        return closest_wp

    def get_next_waypoint_index(self):
        nearest_distance = LARGE_NUMBER
        next_waypoint_index = 0
        for index, waypoint in enumerate(self.waypoints):
            distance = get_distance(self.current_pose.position, waypoint.pose.pose.position)
            if (distance < nearest_distance):
                nearest_distance = distance
                next_waypoint_index = index

        if waypoint_is_feasible(self.current_pose, self.waypoints[next_waypoint_index]):
            return next_waypoint_index
        return next_waypoint_index + 1


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
