import rospy

from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self, vehicle_params):
        self.yaw_controller = YawController(
            wheel_base=vehicle_params['wheel_base'],
            steer_ratio=vehicle_params['steer_ratio'],
            min_speed=vehicle_params['min_speed'],
            max_lat_accel=vehicle_params['max_lat_accel'],
            max_steer_angle=vehicle_params['max_steer_angle'])

        self.throttle_pid = PID(
            kp=5.0, ki=0.0, kd=0.1,
            mn=vehicle_params['decel_limit'],
            mx=vehicle_params['accel_limit'])

        self.steering_filter = LowPassFilter(tau=3.0, ts=1.0)
        self.throttle_filter = LowPassFilter(tau=10.0, ts=1.0)

    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_velocity, time_diff):
        linear_velocity = abs(proposed_linear_velocity)
        velocity_error = linear_velocity - current_velocity

        steering = self.yaw_controller.get_steering(
            linear_velocity,
            proposed_angular_velocity,
            current_velocity)
        steering = self.steering_filter.filt(steering)

        throttle = self.throttle_pid.step(velocity_error, time_diff)
        throttle = self.throttle_filter.filt(throttle)
        throttle = max(0, throttle)

        brake = 0.0

        return throttle, brake, steering

    def reset_pids(self):
        self.throttle_pid.reset()
