from board import SCL, SDA
import board
import busio
import pwmio
from adafruit_motor import motor
import numpy as np
from gpiozero import RotaryEncoder
import os
import sys
import time
from simple_pid import PID


class Motors:
    """
    Simple simulation of a water boiler which can heat up water
    and where the heat dissipates slowly over time
    """


    def __init__(self):
        self.l_motor, self.r_motor = self.get_private_motor()
        self.l_rotor = RotaryEncoder(4, 17, max_steps=0)
        self.r_rotor = RotaryEncoder(5, 13, max_steps=0)
        self.l_rot_p_s = 0
        self.r_rot_p_s = 0
        self.tstart = time.perf_counter()
        self.tprev = 0
        self.tcurr = 0
        self.lprevstep = 0
        self.rprevstep = 0

        # PID Setup
        self.kp = 0.00001
        self.ki = 0.000001
        self.kd = 0.000001

        self.l_pid = PID(self.kp, self.ki, self.kd, setpoint=0)
        self.r_pid = PID(self.kp, self.ki, self.kd, setpoint=0)

        self.l_speed = 0
        self.r_speed = 0

    def get_private_motor(self):
        i2c = busio.I2C(SCL, SDA)

        pwminb1 = pwmio.PWMOut(board.D19)
        pwminb2 = pwmio.PWMOut(board.D26)
        pwmina1 = pwmio.PWMOut(board.D20)
        pwmina2 = pwmio.PWMOut(board.D21)


        motorL= motor.DCMotor(pwmina1, pwmina2)
        motorR = motor.DCMotor(pwminb1, pwminb2)
        motorL.throttle = 0
        motorR.throttle = 0
        return motorL, motorR
    
    def get_motors(self):
        return self.l_motor, self.r_motor


    # calculates the steps since the last timestep
    def get_steps_p_sam(self):
        self.tcurr = time.perf_counter() - self.tstart
        print("current time ", self.tcurr )
        dt = self.tcurr - self.tprev
        l_speed = (self.l_rotor.steps - self.lprevstep) / (dt)
        r_speed = (self.r_rotor.steps - self.rprevstep) / (dt)
        self.lprevstep = self.l_rotor.steps
        self.rprevstep = self.r_rotor.steps
        self.tprev = self.tcurr
        if (l_speed == 0):
            print(self.l_rotor.steps, " " , self.lprevstep , "  " , dt) 
        return l_speed, r_speed


    def update(self,l_motor_power, r_motor_power):
        # Calculate and clamp the left motor throttle
        print("raw speeds : ", self.l_motor.throttle + l_motor_power, " " ,self.r_motor.throttle + r_motor_power)

        self.l_motor.throttle = max(-1, min(self.l_motor.throttle + l_motor_power, 1))

        self.r_motor.throttle = max(-1, min(self.r_motor.throttle + r_motor_power, 1))
        print(self.l_motor.throttle)
        # Time passes
        time.sleep(0.1)

        return self.l_speed, self.r_speed


    def run (self, l_speed_goal, r_speed_goal):
        if l_speed_goal == 0:
            self.l_motor.throttle = 0

        if r_speed_goal == 0:
            self.r_motor.throttle = 0

        if self.l_pid.setpoint != l_speed_goal:
            print("Left Speed is updated to: ", l_speed_goal)
            self.l_pid = PID(self.kp, self.ki, self.kd, setpoint=l_speed_goal)

        if self.r_pid.setpoint != r_speed_goal:
            print("Right Speed is updated to: ", r_speed_goal)
            self.r_pid = PID(self.kp, self.ki, self.kd, setpoint=r_speed_goal)

        self.l_speed, self.r_speed = self.get_steps_p_sam()
        l_power = self.l_pid(self.l_speed)
        r_power = self.r_pid(self.r_speed)
        print(f"l_s: {self.l_speed:.2f}, l_p: {l_power:.2f}, left throttle: {self.l_motor.throttle:.2f}, \nr_s: {self.r_speed:.2f}, r_p: {r_power:.2f}, right throttle: {self.r_motor.throttle:.2f}")
        print("Left setpoint: ", self.l_pid.setpoint)
        print("Right setpoint: ", self.r_pid.setpoint)
        self.l_speed, self.r_speed = self.update(l_power, r_power)

    def setup(self, l_rot_p_sec_goal, r_rot_p_sec_goal):
        kp = 0.00001
        ki = 0.001
        kd = 0.001
        l_pid = PID(kp, ki, kd, setpoint=l_rot_p_sec_goal)
        l_pid.output_limits = (-1, 1)


        r_pid = PID(kp, ki, kd, setpoint=r_rot_p_sec_goal)
        r_pid.output_limits = (-1, 1)



if __name__ == '__main__':
    motors = Motors()

    l_rot, r_rot = motors.get_steps_p_sam()
    kp = 0.0001
    ki = 0.0000
    kd = 0.0000
    l_goal_speed = 500
    r_goal_speed = 500
    l_pid = PID(kp, ki, kd, setpoint=l_goal_speed)
    l_pid.output_limits = (-1, 1)


    r_pid = PID(kp, ki, kd, setpoint=r_goal_speed)
    r_pid.output_limits = (-1, 1)

    l_speed,r_speed = motors.get_steps_p_sam()

    # Keep track of values for plotting
    setpoint, y, x = [], [], []
    start_time = time.time()
    while time.time() - start_time < 20:

        l_power = l_pid(l_speed)
        r_power = r_pid(r_speed)
        l_speed, r_speed = motors.update(l_power, r_power)
        print("l_s ", l_speed, " r_s " , r_speed, " l_p ", l_power, " r_p ", r_power)

        x += [r_rot]
        y += [r_power]
        setpoint += [l_pid.setpoint]

        if time.time() - start_time < 10 and l_pid.setpoint != 900:
            l_pid = PID(kp, ki, kd, setpoint=900)



