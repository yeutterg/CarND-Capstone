<<<<<<< HEAD
#!/usr/bin/env python

from pid import PID
from yaw_controller import YawController
import numpy as np
=======
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
>>>>>>> 2e246617a996d9ef0f69cd5b861e2c90b910dc3f

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
<<<<<<< HEAD

	#  Calculating the steer angle and acceleration by the YawController and PID controller
    def __init__(self, vehicle_mass, fuel_capacity, min_speed, deceleration_limit, acceleration_limit,
                 wheel_base, wheel_radius, steer_ratio, max_lat_accel, max_steer_angle):

        self.deceleration_limit = deceleration_limit
        self.velocity_controller = PID(1.5, 0.01, 0., deceleration_limit, acceleration_limit)
        self.total_mass = vehicle_mass + (fuel_capacity * GAS_DENSITY)
        self.wheel_radius = wheel_radius
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)


    #  Getting the velocity (current and required) and time elapsed from last call
	#  Calculating the velocity correction (acceleration/deceleration) by PID
	#  Return the acceleration (throttle), brake (deceleration) and steer angle 
    def control(self, twist_cmd, current_velocity, time_elapsed):
        vel_error = twist_cmd.twist.linear.x - current_velocity.twist.linear.x
        throttle = self.velocity_controller.step(vel_error, time_elapsed)
        steer = self.yaw_controller.get_steering(twist_cmd.twist.linear.x, twist_cmd.twist.angular.z, current_velocity.twist.linear.x)

        if current_velocity.twist.linear.x < 0.1 and np.isclose(twist_cmd.twist.linear.x, 0.):
            torque = self.total_mass * self.wheel_radius * self.deceleration_limit
            return 0., torque, steer
        elif throttle > 0:
            return throttle, 0., steer
        else:
            torque = self.total_mass * self.wheel_radius * -throttle
            return 0., torque, steer

    def reset(self):
        self.velocity_controller.reset()
=======
    def __init__(self, params):
        self.params = params
	self.pid = PID(kp=5, ki=.5, kd=.5, mn=params['decel_limit'], mx=params['accel_limit'])         
	self.yaw_ctl = YawController(wheel_base=params['wheel_base'], steer_ratio=params['steer_ratio'], min_speed=params['min_speed'], 			 	max_lat_accel=params['max_lat_accel'], max_steer_angle=params['max_steer_angle'])
	self.steer_lpf = LowPassFilter(tau=3, ts=1)
	self.throttle_lpf = LowPassFilter(tau=3, ts=1)
	self.total_mass = (params['vehicle_mass'] + params['fuel_capacity'] * GAS_DENSITY) * params['wheel_radius']

    def control(self, twist, velocity, dt):
        # Return throttle, brake, steer
	v_lin = abs(twist.twist.linear.x)
        v_ang = twist.twist.angular.z
        v_err = v_lin - velocity.twist.linear.x

	steer = self.yaw_ctl.get_steering(v_lin, v_ang, velocity.twist.linear.x)
        steer = self.steer_lpf.filt(steer)

	accel = self.pid.step(v_err, dt)
        accel = self.throttle_lpf.filt(accel)

	if accel > 0.:
            throttle = accel
            brake = 0.
        else:
            throttle = 0.
            decel = -accel
            if decel < self.params['brake_deadband']:
                decel = 0.
            brake = decel * self.total_mass
        return throttle, brake, steer

    def reset(self):
	self.pid.reset()
        

>>>>>>> 2e246617a996d9ef0f69cd5b861e2c90b910dc3f
