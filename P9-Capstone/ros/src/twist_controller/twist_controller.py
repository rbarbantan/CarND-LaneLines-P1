import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, wheel_radius, vehicle_mass, 
                steer_ratio, max_lat_accel, max_steer_angle, decel_limit):

        self.yaw_controller = YawController(wheel_base=wheel_base, steer_ratio=steer_ratio, 
                                            min_speed=0.1, max_lat_accel=max_lat_accel, max_steer_angle=max_steer_angle)
        self.throttle_controller = PID(kp=0.3, ki=0.1, kd=0.0, mn=0.0, mx=0.2)
        self.velocity_lp_filter = LowPassFilter(tau=0.5, ts=0.2)
        self.wheel_radius = wheel_radius
        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.last_time = rospy.get_time()

    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity, dbw_enabled):
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0
        
        steering = self.yaw_controller.get_steering(linear_velocity=proposed_linear_velocity, 
                                                    angular_velocity=proposed_angular_velocity, 
                                                    current_velocity=current_linear_velocity)
        
        velocity_error = proposed_linear_velocity - current_linear_velocity
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(error=velocity_error, sample_time=sample_time)
        brake = 0.0

        if proposed_linear_velocity == 0.0 and current_linear_velocity < 0.1:  # we should stop the car
            throttle = 0
            brake = 700
        elif throttle < 0.1 and velocity_error < 0:  # we should slow down
            throttle = 0
            deceleration = max(velocity_error, self.decel_limit)
            brake = abs(deceleration) * self.vehicle_mass * self.wheel_radius  # torque N*m

        return throttle, brake, steering
