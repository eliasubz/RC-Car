# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# This example uses an Adafruit Stepper and DC Motor FeatherWing to run a DC Motor.
#   https://www.adafruit.com/product/2927

import time

from board import SCL, SDA
import board
import busio
import pwmio
from adafruit_motor import motor

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
motorL.decay_mode = (
    motor.SLOW_DECAY
)
motorR.decay_mode = (
    motor.SLOW_DECAY
)



print("Right")
motorR.throttle = 1 

time.sleep(2)
print("Left")
motorL.throttle = 1
motorR.throttle = 0
time.sleep(2)






print("Forwards")
motorR.throttle = 1
print("throttle:", motorR.throttle)
print("Forwards")
motorL.throttle = 1
print("throttle:", motorL.throttle)
time.sleep(1)


print("Backwards")

print("Forwards slow")
motorR.throttle = -0.5
print("throttle:", motorR.throttle)

print("Forwards slow")
motorL.throttle = -0.5
print("throttle:", motorL.throttle)
time.sleep(1)

print("Forwards")
motorR.throttle = -1
print("throttle:", motorR.throttle)
print("Forwards")
motorL.throttle = -1
print("throttle:", motorL.throttle)
time.sleep(1)




print("Forwards")
motorR.throttle = 0.99
print("throttle:", motorR.throttle)
print("Forwards")
motorL.throttle = -1
print("throttle:", motorL.throttle)
time.sleep(8)
