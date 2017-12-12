#!/usr/bin/env python

import math
import rospy
import tf

from copy import deepcopy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane
from std_msgs.msg import Int32, Bool
from styx_msgs.msg import TrafficLightArray, TrafficLight
import yaml

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
MAXIMUM_ANGLE = math.pi / 4

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

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1) 

        top_speed = rospy.get_param("/waypoint_loader/velocity", None) # km/h
        assert top_speed is not None, "missing parameter?"

        KMH_TO_MPS = 0.27778 # 1 km/h in m/s
        self.target_velocity = top_speed * KMH_TO_MPS

        # for debug purposes, uncomment below to force max velocity
        # METERS_PER_SECOND_MULTIPLE = 0.44704 # 1 mph in m/s
        # TARGET_VELOCITY_MPH = 30.0
        # TARGET_VELOCITY_MPS = TARGET_VELOCITY_MPH * METERS_PER_SECOND_MULTIPLE
        # self.target_velocity = TARGET_VELOCITY_MPS

        rospy.logwarn("top velocity: {0} m/s".format(self.target_velocity))

        # TODO: Add a subscriber for /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.waypoints = None
        self.current_pose = None
        self.current_velocity = None
        self.pose_frame_id = None
        self.next_waypoint_index = None
        self.traffic_light_index = -1

        self.cntt = 0

        self.loop()

    def get_stop_line_waypoints(self, stop_line_positions):
        stop_line_wps = []
        for p in stop_line_positions:
            closest_idx = -1
            min_dist = float('+infinity')
            for wp_idx, wp in enumerate(self.waypoints):
                pos = wp.pose.pose.position
                pos = (pos.x, pos.y)
                dist = get_dist(p, pos)

                if dist < min_dist:
                    closest_idx = wp_idx
                    min_dist = dist

            stop_line_wps.append(closest_idx)

        return stop_line_wps

    def is_ready(self):
        return all((self.waypoints, self.current_pose, self.current_velocity))

    def piecewise_distance(self, waypoints):
        'Walk through the waypoints and calculate distances in between'

        distances = []
        total = 0

        distances.append(0)

        for i in range(1, len(self.waypoints)):
            wp1_pos = self.waypoints[i-1].pose.pose.position
            wp2_pos = self.waypoints[i].pose.pose.position
            dist = get_distance(wp1_pos, wp2_pos)

            total += dist

            distances.append(total)

        return distances

    def update_waypoint_velocities(self, lookahead_waypoints):
        if len(lookahead_waypoints) == 0:
            return

        if self.cntt > 0:
            return

        self.cntt += 1

        dists = self.piecewise_distance(lookahead_waypoints)

        lookahead_waypoints[0].twist.twist.linear.x = \
            min(self.current_velocity.linear.x, self.target_velocity)
        accel = 3.0
        for i in range(1, len(lookahead_waypoints)):
            v0 = lookahead_waypoints[i-1].twist.twist.linear.x
            dx = dists[i]
            vf = math.sqrt(v0*v0 + 2*accel*dx)
            vf = min(self.target_velocity, vf)
            lookahead_waypoints[i].twist.twist.linear.x = vf

        min_vel = lookahead_waypoints[0].twist.twist.linear.x
        max_vel = lookahead_waypoints[-1].twist.twist.linear.x
        rospy.logwarn("min = {0} m/s, max = {1} m/s".format(min_vel, max_vel))

        #if self.traffic_light_index == -1:
        #    for waypoint in lookahead_waypoints:
        #        waypoint.twist.twist.linear.x = self.target_velocity
        #else:
        #    for waypoint in lookahead_waypoints:
        #        waypoint.twist.twist.linear.x = 0

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            if not self.is_ready():
                continue

            lookahead_waypoints = self.get_lookahead_waypoints()

            self.update_waypoint_velocities(lookahead_waypoints)

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

        # setup stop line waypoints for debugging with light classifier
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_ground_truth_cb)
        config_string = rospy.get_param("/traffic_light_config")
        stop_line_positions = yaml.load(config_string)['stop_line_positions']
        self.stop_line_wps = self.get_stop_line_waypoints(stop_line_positions)

        rospy.loginfo('Waypoints Received')

    def traffic_cb(self, waypoint):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_light_index = waypoint
        rospy.logwarn("Receiving traffic light info!")

    def closest_light(self, red_stop_line_wps):

        closest_wp = -1
        min_dist = float('+infinity')

        for stop_line_wp in red_stop_line_wps:
            curr_wp = self.waypoints[stop_line_wp]
            if not waypoint_is_feasible(self.current_pose, curr_wp):
                continue

            dist = get_distance(self.current_pose.position, curr_wp.pose.pose.position)

            if dist < min_dist:
                min_dist = dist
                closest_wp = stop_line_wp

        return closest_wp

    def traffic_ground_truth_cb(self, traffic_light_array):
        red_stop_line_wps = []
        for light_index, l in enumerate(traffic_light_array.lights):
            if l.state == TrafficLight.RED:
                wp_idx = self.stop_line_wps[light_index]
                red_stop_line_wps.append(wp_idx)
        new_traffic_light_index = self.closest_light(red_stop_line_wps)

        if new_traffic_light_index != self.traffic_light_index:
            rospy.logwarn("GT red light waypoint = {0}".format(new_traffic_light_index))
            if new_traffic_light_index >= 0:
                light_wp = self.waypoints[new_traffic_light_index]
                light_line_pos = light_wp.pose.pose.position
                rospy.logwarn("pos = ({0}, {1})".format(light_line_pos.x, light_line_pos.y))

        self.traffic_light_index = new_traffic_light_index

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist

    def get_lookahead_waypoints(self):
        next_waypoint_index = self.get_next_waypoint_index()
        last_waypoint_index = next_waypoint_index + LOOKAHEAD_WPS
        return deepcopy(self.waypoints[next_waypoint_index:last_waypoint_index])

    def get_next_waypoint_index(self):
        nearest_distance = 2e32
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
