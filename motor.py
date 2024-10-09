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
import matplotlib.pyplot as plt
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



    def get_motor():
        i2c = busio.I2C(SCL, SDA)

    #  Motor 1 is channels 9 and 10 with 8 held high.
    # Motor 2 is channels 11 and 12 with 13 held high.
    # Motor 3 is channels 3 and 4 with 2 held high.
    # Motor 4 is channels 5 and 6 with 7 held high.

    # DC Motors generate electrical noise when running that can reset the microcontroller in extreme
    # cases. A capacitor can be used to help prevent this. The demo uses motor 4 because it worked ok
    # in testing without a capacitor.
    # See here for more info: https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/faq#faq-13pca.channels[7].duty_cycle = 0xFFFF


        pwminb1 = pwmio.PWMOut(board.D19)
        pwminb2 = pwmio.PWMOut(board.D26)
        pwmina1 = pwmio.PWMOut(board.D20)
        pwmina2 = pwmio.PWMOut(board.D21)


        motorL= motor.DCMotor(pwmina1, pwmina2)
        motorR = motor.DCMotor(pwminb1, pwminb2)

        return motorL, motorR
    
    # calculates the steps since the last timestep
    def get_steps_p_sam(self):
        return self.l_rotor.steps, self.r_rotor.steps
    

    


    def update(self,l_motor_power, r_motor_power):
        self.l_motor.throttle(l_motor_power)
        self.r_motor.throttle(r_motor_power)

        # Time passes 
        time.sleep(0.2)

        return self.get_steps_p_sam()
    

    # 
    def setup(self, l_rot_p_sam, r_rot_p_sam_goal):
        self.l_pid = PID(0, 0, 0, setpoint=l_rot_p_sam)
        l_pid.output_limits = (-1, 1)

        r_pid = PID(0, 0, 0, setpoint=r_rot_p_sam_goal)
        r_pid.output_limits = (-1, 1)







if __name__ == '__main__':
    motors = Motors()
    
    l_rot, r_rot = motors.get_steps_p_sam()


    l_pid = PID(0, 0, 0, setpoint=5)
    l_pid.output_limits = (-1, 1)

    r_pid = PID(0, 0, 0, setpoint=5)
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
        l_rot, r_rot = motors.update(l_power, r_power)

        x += [current_time - start_time]
        y += [r_power]
        setpoint += [l_pid.setpoint]

        if current_time - start_time > 1:
            l_pid.setpoint = 100

        last_time = current_time

    plt.plot(x, y, label='measured')
    plt.plot(x, setpoint, label='target')
    plt.xlabel('time')
    plt.ylabel('temperature')
    plt.legend()
    if os.getenv('NO_DISPLAY'):
        # If run in CI the plot is saved to file instead of shown to the user
        plt.savefig(f"result-py{'.'.join([str(x) for x in sys.version_info[:2]])}.png")
    else:
        plt.show()
