import numpy as np
import control as ct
import matplotlib.pyplot as plt
import math

class Controller():
    def __init__(self, dt:None):
        
        self.dt = dt
        self.rho = 1000
        self.A = 0.5*0.25
        self.I = 1
        self.angularVelPrev = 0

    def ARE(self, A,B,Q,R):
     
        # Number of iterations to converge P
        N = 50

        P = [None] * (N+1)

        P[N] = Q

        for i in range(N, 0, -1):

            P[i-1] = Q + (A.T @ P[i] @ A) - ((A.T @ P[i] @ B) @ (np.linalg.pinv(R + ( B.T @ P[i] @ B))) @ (B.T @ P[i] @ A))

        
        K = -np.linalg.pinv(R) @ B.T @ P[N-1]

        return K
    
    def getB(self, heading, dt):
        # Defining the matrix B which describes how the state changes due to control inputs (3x2)
        # Control inputs are linear velocity and angular velocity
        # dist = current_dist - v*dt
        # heading = heading_current + ang_vel*dt
        B = np.array(
            [
                [-dt,0],
                [0, dt]
            ]
        )
        return B
    
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

    def LQR(self, desiredState, currentState, heading, linearVelocity, angularVelocity):

        # Defining the state space A matrix to define the state of the system
        # State of the system are distance to target, heading
        A =  np.array(
            [
                [1.0,0],
                [0,1.0]
            ]
        )

        B = self.getB(heading, self.dt)

        # Defining matrix Q which describes the cost of system state error (2x2)
        # Describes the cost of not reaching the desired dist to target, and heading

        Q = np.array(
            [
                [1.0,0],
                [0,100.0]
            ]
        )

        # Defining matrix R which describes the cost of using actuators (2x2) or square matrix corresponding to the number of control inputs
        # Describes the cost of input regarding linear and angular velocity

        R = np.array(
            [
                [10.0,0],
                [0,1.0]
            ]
        )

        stateError = currentState - desiredState
        print(f'State Error = {stateError}')

        stateErrorMagnitude = np.linalg.norm(stateError)

        K = self.ARE(A,B,Q,R)

        uStar = K @ stateError

        print(f'Optimal Control Input = {uStar}')

        alpha = (-angularVelocity + uStar[1]*linearVelocity) / (self.dt)
        
        sintheta = (self.I*alpha)/(self.rho*self.A*linearVelocity)
        # print(sintheta)
        if abs(sintheta) > 1:
            sintheta = 1 / (sintheta / abs(sintheta))

        rudderCmd = math.asin(sintheta)

        thrustCmd = uStar[0] / self.dt

        return uStar[1], thrustCmd





