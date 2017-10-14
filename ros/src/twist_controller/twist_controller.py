from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
from rospy import logwarn

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, params):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base=params['wheel_base'], 
                                            steer_ratio=params['steer_ratio'] ,
                                            min_speed=params['min_speed'], 
                                            max_lat_accel=params['max_lat_accel'], 
                                            max_steer_angle=params['max_steer_angle'])
        self.set_controllers()

    def control(self, target_speed, target_angular_speed, current_speed, time_elapsed):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(target_speed, target_angular_speed, current_speed)
        steer = self.lowpass_steer.filt(steer)

        cte = target_speed - current_speed
        throttle = self.pid_throttle.step(cte, time_elapsed)
        brake = self.pid_brake.step(-cte, time_elapsed)

        # return 0.3, 0., steer
        return throttle, brake, steer

    def set_controllers(self):
        #Rough estimate for PID controller
        self.pid_throttle = PID(0.35,0.0,0.0,0.0,1.0)
        self.pid_brake = PID(0.5,0.0,0.0,0.0,1.0)
        #Low pass filter for steering cmd
        self.lowpass_steer = LowPassFilter(0.2,1.0)