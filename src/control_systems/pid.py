import numpy as np
import control as ct
import matplotlib.pyplot as plt

class Controller():
    def __init__(self, dt:None):
        
        self.dt = dt

    
    def PID(self,Kp,Ki,Kd,feedback:None,target:None,integral:None,error_prev:None,feedback_prev:None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.smoothing = 0.01

        self.feedback = feedback
        self.target = target
        self.error = target - feedback

        if integral is not None:
            self.integral = integral
        else:
            self.integral = 0

        if error_prev is not None:
            self.error_prev = error_prev
        else:
            self.error_prev = self.error

        if feedback_prev is not None:
            self.feedback_prev = feedback_prev
        else:
            self.feedback_prev = feedback

        self.proportion = self.error * self.Kp

        self.integral += (self.error + self.error_prev) * 0.5 * self.Ki * self.dt

        self.derivative = (-2 * self.Kd * (self.feedback_prev - self.feedback) + (2 * self.smoothing - self.dt) * self.Kd / (2 * self.smoothing + self.dt))

        self.output = (self.proportion + self.integral + self.derivative)

        return self.output, self.integral, self.error, self.feedback



