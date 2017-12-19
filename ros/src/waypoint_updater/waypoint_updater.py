#!/usr/bin/env python

import math
import rospy
import tf

from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane
from std_msgs.msg import Int32
from styx_msgs.msg import TrafficLightArray, TrafficLight
import yaml

import numpy as np
from lowpass import LowPassFilter
from bisect import bisect_right

LARGE_NUMBER = 2e32
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish
MAXIMUM_ANGLE = math.pi / 4
MAXIMUM_DECELERATION = 0.5
STOPPING_DISTANCE = 5.0
MPH_TO_MPS = 0.44704

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
    """
    Set the linear velocity of a given waypoint
    """
    waypoint.twist.twist.linear.x = velocity

def decelerate_waypoints_to_target(waypoints, target_index):
    """
    Set the linear velocity of the target and
    all following waypoints to zero, and smoothly
    decelerate all preceding waypoints
    """
    target_waypoint = waypoints[target_index]

    for index, waypoint in enumerate(waypoints):
        if index > target_index:
            velocity = 0.
        else:
            distance = get_distance(waypoint.pose.pose.position, target_waypoint.pose.pose.position)
            distance = max(0., distance-STOPPING_DISTANCE)
            velocity = math.sqrt(2 * MAXIMUM_DECELERATION * distance)
            if velocity < 1.:
                velocity = 0.
        set_waypoint_linear_velocity(waypoint, velocity)

def get_distance(p1, p2):
    """
    Calculate the Euclidean distance between 2 points
    """
    x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
    return math.sqrt(x*x + y*y + z*z)

class WaypointUpdater(object):
    """
    Publish waypoints from the car's current position
    to some distance ahead.
    """
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.loginfo('Waypoint Updater Initialized')

        top_speed = rospy.get_param("/waypoint_loader/velocity", None)
        assert top_speed is not None, "Missing velocity parameter"

        self.target_velocity = top_speed * MPH_TO_MPS

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
        """
        Determine whether it is safe to begin publishing waypoints
        """
        return all((self.waypoints, self.current_pose))

    def loop(self):
        """
        Determine indices of the next N waypoints ahead.
        If a red light is detected and is within our
        lookahead window, decelerate the waypoint velocities,
        otherwise set the waypoint velocities to the target velocity
        """
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            if not self.is_ready():
                continue

            next_waypoint_index = self.get_next_waypoint_index()
            last_waypoint_index = next_waypoint_index + LOOKAHEAD_WPS
            lookahead_waypoints = deepcopy(self.waypoints[next_waypoint_index:last_waypoint_index])
            relative_red_light_index = self.traffic_light_index - next_waypoint_index

            # if red light has been detected and is in range, decelerate
            if relative_red_light_index > -1 and relative_red_light_index < LOOKAHEAD_WPS:
                decelerate_waypoints_to_target(lookahead_waypoints, relative_red_light_index)
            else:
                for waypoint in lookahead_waypoints:
                    set_waypoint_linear_velocity(waypoint, self.target_velocity)

            self.publish(lookahead_waypoints)

    def publish(self, waypoints):
        """
        Publish the given waypoints
        """
        lane = Lane()
        lane.header.frame_id = self.pose_frame_id
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        """
        Update the current pose and pose frame id
        """
        self.current_pose = msg.pose
        self.pose_frame_id = msg.header.frame_id

    def waypoints_cb(self, msg):
        """
        Save the waypoints published by the waypoint
        loader node. Called once.
        """
        self.waypoints = msg.waypoints
        rospy.loginfo('Waypoints Received')

        config_string = rospy.get_param("/traffic_light_config")
        stop_line_positions = yaml.load(config_string)['stop_line_positions']
        self.stop_line_wps = self.get_stop_line_waypoints(stop_line_positions)

    def traffic_cb(self, waypoint):
        """
        Set the latest published traffic light indices
        """
        self.traffic_light_index = waypoint.data
        rospy.logwarn("Receiving traffic light info!")

    def traffic_ground_truth_cb(self, traffic_light_array):
        """
        Helper function to check detected traffic
        lights against ground truth
        """
        if not self.stop_line_wps or not self.is_ready():
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
        """
        Determine the waypoint indices associated
        with the stopping lines
        """
        stop_line_wps = []

        for pos in stop_line_positions:
            stop_line = PoseStamped()
            stop_line.pose.position.x = float(pos[0])
            stop_line.pose.position.y = float(pos[1])

            closest_idx = -1
            min_dist = LARGE_NUMBER
            for index, waypoint in enumerate(self.waypoints):
                dist = get_distance(stop_line.pose.position, waypoint.pose.pose.position)
                if dist < min_dist:
                    closest_idx = index
                    min_dist = dist

            stop_line_wps.append(closest_idx)
        return stop_line_wps

    def closest_light(self, red_stop_line_wps):
        """
        Get the nearest traffic light
        """
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
        """
        Find the nearest waypoint index that is
        ahead of the vehicle's current position
        """
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
