# SPDX-FileCopyrightText: 2019 Mikey Sklar for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import os
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn


def voltage_to_distance(voltage):
    # Define the reference data points (voltage, distance)
    data = [
        (2.75, 15),   # 2.75 V -> 15 cm
        (2.0, 30),    # 2.0 V -> 30 cm
        (1.5, 40),    # 1.5 V -> 40 cm
        (1.2, 50),
        (0.93, 60),    # 1.1 V -> 60 cm
        (0.83, 70),
        (0.73, 80),
        (0.64, 90),
        (0.58, 100),

 # 0.75 V -> 90 cm
        (0.5, 130)    # 0.5 V -> 130 cm
    ]



    if voltage < data[-1][0]:
        v1, d1 = data[-2]  # (0.75, 90)
        v2, d2 = data[-1]  # (0.5, 130)
        # Continue the linear relationship
        slope = (d2 - d1) / (v2 - v1)
        distance = d2 + slope * (voltage - v2)
        return distance

    if voltage > data[0][0]:
        v1, d1 = data[0]  # (0.75, 90)
        v2, d2 = data[1]  # (0.5, 130)
        # Continue the linear relationship
        slope = (d2 - d1) / (v2 - v1)
        distance = d2 + slope * (voltage - v2)
        return distance



    # Interpolate between the given data points
    for i in range(len(data) - 1):
        v1, d1 = data[i]
        v2, d2 = data[i + 1]

        if v2 <= voltage <= v1:  # Find the correct segment
            # Perform linear interpolation
            distance = d1 + (d2 - d1) * (voltage - v1) / (v2 - v1)
            return distance

    return None  # If voltage doesn't fall within any range


# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

print('Raw ADC Value: ', chan0.value)
print('ADC Voltage: ' + str(chan0.voltage) + 'V')

last_read = 0       # this keeps track of the last potentiometer value
tolerance = 250     # to keep from being jittery we'll only change
                    # volume when the pot has moved a significant amount
                    # on a 16-bit ADC

def remap_range(value, left_min, left_max, right_min, right_max):
    # this remaps a value from original (left) range to new (right) range
    # Figure out how 'wide' each range is
    left_span = left_max - left_min
    right_span = right_max - right_min

    # Convert the left range into a 0-1 range (int)
    valueScaled = int(value - left_min) / int(left_span)
    # Convert the 0-1 range into a value in the right range.
    return int(right_min + (valueScaled * right_span))

while True:
    # we'll assume that the pot didn't move
    trim_pot_changed = False
    # print("Loop has passed")

    volt = chan0.voltage
    print('Raw ADC Value: ',volt)
    print('Distance: ', voltage_to_distance(volt))



    # how much has it change
    # hang out and do nothing for a half second
    time.sleep(0.5)

