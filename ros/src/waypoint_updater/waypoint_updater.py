#!/usr/bin/env python

import math
import rospy
import tf

from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane
from std_msgs.msg import Int32
from styx_msgs.msg import TrafficLightArray, TrafficLight

LARGE_NUMBER = 2e32
MAXIMUM_ANGLE = math.pi / 4
MAXIMUM_DECELERATION = 0.8
STOPPING_DISTANCE = 8.0
MPH_TO_MPS = 0.44704

class CircularBuffer:
    """
    Convenience class to wrap the waypoints
    to abstract away wrap around when slicing.
    """
    def __init__(self, l):
        self.l = l
    def __getitem__(self, idx):
        if isinstance(idx, slice):
            (start, stop) = (idx.start, idx.stop)
            newlist = []
            for i in range(start, stop):
                newlist.append(self.l[i % len(self.l)])
            return newlist
        return self.l[idx % len(self.l)]
    def __len__(self):
        return len(self.l)
    def __iter__(self):
        for x in self.l:
            yield x

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

def get_waypoint_linear_velocity(waypoint):
    """
    Get the linear velocity of a given waypoint
    """
    return waypoint.twist.twist.linear.x

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

def accelerate_waypoints_to_target(position, waypoints, v0, vf):
    """
    Set upcoming waypoints to smoothly accelerate up to the target velocity.
    """
    for wp in waypoints:
        distance = get_distance(position, wp.pose.pose.position)
        velocity = math.sqrt(v0*v0 + 2 * (MAXIMUM_DECELERATION) * distance)
        velocity = min(velocity, vf)
        set_waypoint_linear_velocity(wp, velocity)

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

        self.LOOKAHEAD_WPS = rospy.get_param('~lookahead_waypoints', None)
        assert self.LOOKAHEAD_WPS is not None, "Missing lookahead"

        self.target_velocity = top_speed * MPH_TO_MPS

        rospy.loginfo("Lookahead distance: {0} waypoints".format(self.LOOKAHEAD_WPS))
        rospy.loginfo("Top velocity: {0} m/s".format(self.target_velocity))

        self.waypoints = None
        self.current_pose = None
        self.pose_frame_id = None
        self.linear_velocity = None
        self.prev_next_wp_index = None
        self.traffic_light_index = -1

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def is_ready(self):
        """
        Determine whether it is safe to begin publishing waypoints
        """
        return all((self.waypoints, self.current_pose, self.linear_velocity))

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
            last_waypoint_index = next_waypoint_index + self.LOOKAHEAD_WPS
            lookahead_waypoints = self.waypoints[next_waypoint_index:last_waypoint_index]
            red_light_index = (self.traffic_light_index - next_waypoint_index) % len(self.waypoints)

            # if red light has been detected and is in range, decelerate
            if self.traffic_light_index != -1 and red_light_index < self.LOOKAHEAD_WPS/3:
                decelerate_waypoints_to_target(lookahead_waypoints, red_light_index)
                self.prev_next_wp_index = None
            else:
                if self.prev_next_wp_index is None:
                    accelerate_waypoints_to_target(
                        self.current_pose.position, lookahead_waypoints,
                        self.linear_velocity, self.target_velocity)
                else:
                    num_wps_todo = (next_waypoint_index - self.prev_next_wp_index) % len(self.waypoints)
                    if num_wps_todo > 10:
                        self.prev_next_wp_index = None
                        rospy.logwarn('reset...')
                        continue
                    last_wp = lookahead_waypoints[-1 - num_wps_todo]
                    v0 = get_waypoint_linear_velocity(last_wp)
                    if num_wps_todo != 0:
                        accelerate_waypoints_to_target(
                            last_wp.pose.pose.position, lookahead_waypoints[-num_wps_todo:],
                            v0, self.target_velocity)

                self.prev_next_wp_index = next_waypoint_index

                rospy.logwarn('{}'.format(lookahead_waypoints[0].twist.twist.linear.x))

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
        self.waypoints = CircularBuffer(msg.waypoints)
        rospy.loginfo('# Waypoints Received: {}'.format(len(self.waypoints)))

    def traffic_cb(self, waypoint):
        """
        Set the latest published traffic light indices
        """
        self.traffic_light_index = waypoint.data

    def current_velocity_cb(self, msg):
        self.linear_velocity = msg.twist.linear.x

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
