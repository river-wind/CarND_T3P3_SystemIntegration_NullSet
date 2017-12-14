#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped

from twist_controller import TwistController

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
        rospy.loginfo('DBW Node Initialized')

        vehicle_params = {
            'vehicle_mass': rospy.get_param('~vehicle_mass', 1736.35), # kg
            'fuel_capacity': rospy.get_param('~fuel_capacity', 13.5), # kg/gal
            'brake_deadband': rospy.get_param('~brake_deadband', .1),
            'decel_limit': rospy.get_param('~decel_limit', -5),
            'accel_limit': rospy.get_param('~accel_limit', 1.), # 1 g?
            'wheel_radius': rospy.get_param('~wheel_radius', 0.2413),
            'wheel_base': rospy.get_param('~wheel_base', 2.8498),
            'steer_ratio': rospy.get_param('~steer_ratio', 14.8),
            'max_lat_accel': rospy.get_param('~max_lat_accel', 3.),
            'max_steer_angle': rospy.get_param('~max_steer_angle', 8.),
            'min_speed': 0.0,
        }

        self.dbw_enabled = False
        self.reset_flag = True
        self.current_velocity = None
        self.current_twist_command = None
        self.previous_timestamp = rospy.get_time()

        self.controller = TwistController(vehicle_params)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        
        self.loop()

    def is_ready(self):
        return all((self.dbw_enabled, self.current_velocity, self.current_twist_command,))

    def loop(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            rate.sleep()

            current_timestamp = rospy.get_time()
            time_diff = current_timestamp - self.previous_timestamp
            self.previous_timestamp = current_timestamp

            if self.is_ready():
                throttle, brake, steering = self.controller.control(
                    self.current_twist_command.linear.x,
                    self.current_twist_command.angular.z,
                    self.current_velocity.linear.x,
                    time_diff)

                #rospy.logwarn("throttle = {0}".format(throttle))
                self.publish(throttle, brake, steering)
            else:
                self.controller.reset_pids()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data
        rospy.loginfo('DBW Enabled')

    def twist_cmd_cb(self, msg):
        self.current_twist_command = msg.twist

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist

if __name__ == '__main__':
    DBWNode()
