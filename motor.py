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
        self.l_motor, self.r_motor = self.get_motor()
        self.l_rotor = RotaryEncoder(4, 17)
        self.r_rotor = RotaryEncoder(5, 13)
        self.l_rot_p_s = 0
        self.r_rot_p_s = 0
        self.tstart = time.perf_counter()
        self.tprev = 0
        self.tcurr = 0


    def get_motor(self):
        i2c = busio.I2C(SCL, SDA)

        pwminb1 = pwmio.PWMOut(board.D19)
        pwminb2 = pwmio.PWMOut(board.D26)
        pwmina1 = pwmio.PWMOut(board.D20)
        pwmina2 = pwmio.PWMOut(board.D21)


        motorL= motor.DCMotor(pwmina1, pwmina2)
        motorR = motor.DCMotor(pwminb1, pwminb2)
        motorL.throttle = 1

        return motorL, motorR


    # calculates the steps since the last timestep
    def get_steps_p_sam(self):

        return self.l_rotor.steps, self.r_rotor.steps


    def update(self,l_motor_power, r_motor_power):
        self.l_motor.throttle = l_motor_power
        self.r_motor.throttle = r_motor_power

        # Time passes
        time.sleep(0.2)

        return self.get_steps_p_sam()
    #
    def setup(self, l_rot_p_sam, r_rot_p_sam_goal):
        self.l_pid = PID(0.1, 0, 0, setpoint=l_rot_p_sam)
        l_pid.output_limits = (-1, 1)

        r_pid = PID(1, 0, 0, setpoint=r_rot_p_sam_goal)
        r_pid.output_limits = (-1, 1)



if __name__ == '__main__':
    motors = Motors()

    l_rot, r_rot = motors.get_steps_p_sam()
    kp = 0.25
    ki = 0.01
    kd = 0
    l_pid = PID(kp, ki, kd, setpoint=20)
    l_pid.output_limits = (-1, 1)


    r_pid = PID(kp, ki, kd, setpoint=20)
    r_pid.output_limits = (-1, 1)

    start_time = time.time()
    last_time = start_time

    # Keep track of values for plotting
    setpoint, y, x = [], [], []

    while time.time() - start_time < 10:


        current_time = time.time()
        dt = current_time - last_time

        l_power = l_pid(l_rot)
        r_power = r_pid(r_rot)
        print("r_rot: ", r_rot)
        l_rot, r_rot = motors.update(l_power, r_power)
        print("l rotations ", l_rot, " r_rotations " , r_rot)

        x += [r_rot]
        y += [r_power]
        setpoint += [l_pid.setpoint]

        if current_time - start_time > 1:
            l_pid.setpoint = 20

        last_time = current_time

print("x \n", x)
print("y \n", y)
print("setpoint \n", setpoint)
