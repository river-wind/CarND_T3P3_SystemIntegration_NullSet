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

import numpy as np
import scipy.optimize
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

LOOKAHEAD_WPS = 150 # Number of waypoints we will publish. You can change this number
MAXIMUM_ANGLE = math.pi / 4

MAX_ACCEL = 10 # m/s^2
MAX_JERK  = 10 # m/s^2

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

def velocity_search(start, end, t0, tf, cost_fn):
    step = (tf - t0) / 20.0

    min_cost = float('+infinity')
    best_coeffs = None
    best_time = None
    curr_time = t0
    while curr_time <= tf:
        coeffs = list(JMT(start, end, curr_time))
        curr_cost = cost_fn(coeffs, curr_time)
        if curr_cost < min_cost:
            best_coeffs = coeffs
            min_cost = curr_cost
            best_time = curr_time
        curr_time += step

    return (best_coeffs, best_time, min_cost)

def JMT(start, end, T):
    """
    Calculates Jerk Minimizing Trajectory for start, end and T.
    """
    a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
    c_0 = a_0 + a_1 * T + a_2 * T**2
    c_1 = a_1 + 2* a_2 * T
    c_2 = 2 * a_2

    A = np.array([
            [  T**3,   T**4,    T**5],
            [3*T**2, 4*T**3,  5*T**4],
            [6*T,   12*T**2, 20*T**3],
        ])
    B = np.array([
            end[0] - c_0,
            end[1] - c_1,
            end[2] - c_2
        ])
    a_3_4_5 = np.linalg.solve(A,B)
    alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4_5])
    return alphas[::-1]

def check_vt(coeffs, T):
    steps = T / 20.0
    curr_t = 0
    while curr_t <= T:
        if np.polyval(coeffs, curr_t) < 0:
            rospy.logerr("exploded!")
            return
        curr_t += steps

def cost_jerk_accel(coeffs, T):
    vt = np.polyder(coeffs)
    at = np.polyder(vt)
    jt = np.polyder(at)

    steps = T / 20.0
    curr_t = 0

    penalty = 0
    while curr_t <= T:
        if np.polyval(at, curr_t) >= MAX_ACCEL - 2:
            penalty += 1000
        if np.polyval(jt, curr_t) >= MAX_JERK - 2:
            penalty += 1000
        if np.polyval(vt, curr_t) < 0:
            penalty += 100
        curr_t += steps

    penalty += T * 20

    return penalty

def approx_waypoint_times(dists, coeffs, T):
    waypoint_times = []
    for d in dists:
        zcoeffs = np.copy(coeffs)
        zcoeffs[-1] -= d
        try:
            t0 = scipy.optimize.brentq(
                lambda t: np.polyval(zcoeffs, t), 0, T)
            waypoint_times.append(t0)
        except ValueError:
            break

    return waypoint_times

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.loginfo('Waypoint Updater Initialized')

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

        self.waypoints = None
        self.current_pose = None
        self.current_velocity = None
        self.pose_frame_id = None
        self.next_waypoint_index = None
        self.traffic_light_index = -1
        self.dbw_enabled = False

        self.accel_estimate = 0
        self.prev_velocity_time = 0
        self.accel_filter = LowPassFilter(tau=10.0, ts=2.0)

        self.decel_thresh = False

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

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

    def piecewise_distance(self, curr_pose, waypoints):
        'Walk through the waypoints and calculate distances in between'

        dist = get_distance(curr_pose.position, waypoints[0].pose.pose.position)

        total = dist

        tot_distances = [total]
        distances = [dist]

        for i in range(1, len(waypoints)):
            wp1_pos = waypoints[i-1].pose.pose.position
            wp2_pos = waypoints[i].pose.pose.position
            dist = get_distance(wp1_pos, wp2_pos)

            total += dist

            distances.append(dist)
            tot_distances.append(total)

        return (distances, tot_distances)

    def update_waypoint_velocities(self, lookahead_waypoints, wp0, wpf):
        if len(lookahead_waypoints) == 0:
            return

        (dists, tot_dists) = self.piecewise_distance(self.current_pose, lookahead_waypoints)

        curr_pos = self.current_pose.position
        v0 = self.current_velocity.linear.x

        #rospy.logwarn("a = {0}".format(self.accel_estimate))

        if self.traffic_light_index != -1:
            if wp0 <= self.traffic_light_index <= wpf:
                # light coming up
                #rospy.logwarn("I saw the sign! = {0}".format(wp0))
                stop_line_offset = self.traffic_light_index - wp0
                dist_from_stop_line = tot_dists[stop_line_offset] - 2.4
                rospy.logwarn("*******************************")
                rospy.logwarn("dist from stop line = {0} m".format(dist_from_stop_line))
                if dist_from_stop_line < 2:
                    del lookahead_waypoints[:]
                    return
                start = [0, v0, self.accel_estimate]
                end   = [dist_from_stop_line-1, 0, 0]
                (coeffs, T, cost) = velocity_search(start, end, 0.5, 15.0, cost_jerk_accel)
                rospy.logwarn("d = {0}, v0 = {1}, a = {2}, T = {3}, c = {4}".format(
                    dist_from_stop_line, v0, self.accel_estimate, T, coeffs))
                #rospy.logwarn("coeffs = {0}, T = {1}".format(coeffs, T))
                Ts = approx_waypoint_times(tot_dists, coeffs, T)
                Ts = Ts[1:]
                #latency = 0.1
                vt = np.polyder(coeffs)
                #check_vt(vt, T)
                for (i, t) in enumerate(Ts):
                    curr_v = np.polyval(vt, t)
                    lookahead_waypoints[i].twist.twist.linear.x = curr_v
                del lookahead_waypoints[len(Ts):]
            else:
                pass
                # can't see yet

        """
        accel = 3.0
        if self.traffic_light_index != -1:
            if wp0 <= self.traffic_light_index <= wpf:
                stop_line_offset = self.traffic_light_index - wp0
                dist_from_stop_line = tot_dists[stop_line_offset]
                rospy.logwarn("*******************************")
                rospy.logwarn("dist from stop line = {0} m".format(dist_from_stop_line))
                if dist_from_stop_line < 70:
                    prop_accel = -0.5*v0*v0 / dist_from_stop_line
                    if self.decel_thresh:
                        accel = prop_accel
                    else:
                        radicand = v0*v0 + 2*prop_accel*dist_from_stop_line
                        if radicand >= 0:
                            time = (-v0 + math.sqrt(radicand)) / prop_accel
                            if time > 6:
                                pass
                            else:
                                self.decel_thresh = True

        for i in range(len(lookahead_waypoints)):
            dx = dists[i]
            radicand = v0*v0 + 2*accel*dx
            vf = math.sqrt(radicand) if radicand > 0 else 0
            vf = min(self.target_velocity, vf)
            lookahead_waypoints[i].twist.twist.linear.x = vf
            v0 = vf
        """

        # Full path to stop light
        if self.traffic_light_index != -1:
            if wp0 <= self.traffic_light_index <= wpf:
                rospy.logwarn("wp0 = {0}, wp light = {1}, wpf = {2}".format(wp0, self.traffic_light_index, wpf))
                vels = [wp.twist.twist.linear.x for wp in lookahead_waypoints]
                rospy.logwarn(", ".join(["({:.2f}, {:.2f})".format(v, d) for (v, d) in zip(vels, tot_dists)]))
                #rospy.logwarn("{0}".format(vels))
                rospy.logwarn("*******************************")

        #min_vel = lookahead_waypoints[0].twist.twist.linear.x
        #max_vel = lookahead_waypoints[-1].twist.twist.linear.x

        if self.dbw_enabled:
            '''
            rospy.logwarn("curr_vel = {0} m/s, min = {1} m/s, max = {2} m/s, len = {3}".format(
            self.current_velocity.linear.x, min_vel, max_vel, len(lookahead_waypoints)))
            '''

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            if not self.is_ready():
                continue

            next_waypoint_index = self.get_next_waypoint_index()
            last_waypoint_index = next_waypoint_index + LOOKAHEAD_WPS
            #rospy.logwarn("wp_s = {0}, wp_f = {1}".format(next_waypoint_index, last_waypoint_index))
            lookahead_waypoints = deepcopy(self.waypoints[next_waypoint_index:last_waypoint_index])

            self.update_waypoint_velocities(
                lookahead_waypoints, next_waypoint_index, last_waypoint_index-1)

            self.publish(lookahead_waypoints)

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = self.pose_frame_id
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        #rospy.logwarn("got pose!")
        self.current_pose = msg.pose
        self.pose_frame_id = msg.header.frame_id

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints

        config_string = rospy.get_param("/traffic_light_config")
        stop_line_positions = yaml.load(config_string)['stop_line_positions']
        self.stop_line_wps = self.get_stop_line_waypoints(stop_line_positions)

        # setup stop line waypoints for debugging with light classifier
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_ground_truth_cb)

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
            dist = get_distance(self.current_pose.position, curr_wp.pose.pose.position)

            if dist < 30:
                return stop_line_wp

            if not waypoint_is_feasible(self.current_pose, curr_wp):
                continue

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
            self.decel_thresh = False
            rospy.logwarn("GT red light waypoint = {0}".format(new_traffic_light_index))
            if new_traffic_light_index >= 0:
                light_wp = self.waypoints[new_traffic_light_index]
                light_line_pos = light_wp.pose.pose.position
                rospy.logwarn("pos = ({0}, {1})".format(light_line_pos.x, light_line_pos.y))

        self.traffic_light_index = new_traffic_light_index

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data;
        pass

    def current_velocity_cb(self, msg):
        if self.current_velocity is not None:
            prev_velocity = self.current_velocity.linear.x
        else:
            prev_velocity = 0
        prev_time = self.prev_velocity_time

        self.prev_velocity_time = rospy.Time.now().nsecs
        self.current_velocity = msg.twist

        dv = self.current_velocity.linear.x - prev_velocity
        dt = (self.prev_velocity_time - prev_time) / 1e9

        curr_accel = dv / dt
        self.accel_estimate = self.accel_filter.filt(curr_accel)
        #rospy.logwarn("accel estimate = {0}".format(self.accel_estimate))

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
