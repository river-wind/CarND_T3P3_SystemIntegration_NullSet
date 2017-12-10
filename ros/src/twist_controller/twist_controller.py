
from pid import PID
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


#input params from dbw node:
#vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        # receive all the properties sent over by the dbw_node, needed by the yawController and velocity PID
        max_lat_accel = kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']
        wheel_base = kwargs['wheel_base']
        steer_ratio = kwargs['steer_ratio']
        decel_limit = kwargs['decel_limit']
        accel_limit = kwargs['accel_limit']
       
        #use the yaw controller class provided to calculate the steering value
        self.yaw_controller = YawController(wheel_base = wheel_base, 
                                           steer_ratio = steer_ratio,
                                           min_speed = 0., 
                                           max_lat_accel = max_lat_accel, 
                                           max_steer_angle = max_steer_angle)

        self.throttle_pid = PID(.5, 0., 0., decel_limit, accel_limit)
    
        self.throttle = 1.0
        self.brake = 0.0

        self.last_timestamp = rospy.get_time()



    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        command_linear_velocity = kwargs['command_linear_velocity']
        command_angular_velocity = kwargs['command_angular_velocity']
        current_linear_velocity = kwargs['current_linear_velocity']

        current_timestamp = rospy.get_time()
        duration = current_timestamp - self.last_timestamp
        self.last_timestamp - current_timestamp

        #use the pid controller to manage the throttle
        self.throttle = self.throttle_pid.step(command_linear_velocity - current_linear_velocity, 0.05)

        #use the yawController to handle steering calculation
        steering_angle = self.yaw_controller.get_steering(command_linear_velocity, command_angular_velocity, current_linear_velocity)

        # Return throttle, brake, steer
        return self.throttle, self.brake, steering_angle   #1., 0., 0.
