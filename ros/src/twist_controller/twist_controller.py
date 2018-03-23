from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
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
        

