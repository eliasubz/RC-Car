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

class Infra:
    def __init__(self):
        # Initialize SPI bus and MCP3008
        self.spi = busio.SPI(clock=board.SCK,  MISO=board.MISO, MOSI=board.MOSI)
        self.cs = digitalio.DigitalInOut(board.D12)
        self.mcp = MCP.MCP3008(self.spi, self.cs)
        self.channel = AnalogIn(self.mcp, MCP.P0)

    def voltage_to_distance(self, voltage):
        """Convert voltage to distance using a reference table and linear interpolation."""
        # Reference data points (voltage, distance)
        data = [
            (2.75, 15),  # 2.75 V -> 15 cm
            (2.30, 20),
            (2.0, 30),   # 2.0 V -> 30 cm
            (1.5, 40),   # 1.5 V -> 40 cm
            (1.2, 50),   # 1.2 V -> 50 cm
            (0.93, 60),  # 0.93 V -> 60 cm
            (0.83, 70),  # 0.83 V -> 70 cm
            (0.73, 80),  # 0.73 V -> 80 cm
            (0.64, 90),  # 0.64 V -> 90 cm
            (0.58, 100), # 0.58 V -> 100 cm
            (0.5, 130)   # 0.5 V -> 130 cm
        ]

        # Handle voltages below the lowest point
        if voltage < data[-1][0]:
            v1, d1 = data[-2]
            v2, d2 = data[-1]
            slope = (d2 - d1) / (v2 - v1)
            return d2 + slope * (voltage - v2)

        # Handle voltages above the highest point
        if voltage > data[0][0]:
            v1, d1 = data[0]
            v2, d2 = data[1]
            slope = (d2 - d1) / (v2 - v1)
            return d2 + slope * (voltage - v2)

        # Interpolate between reference points
        for i in range(len(data) - 1):
            v1, d1 = data[i]
            v2, d2 = data[i + 1]
            if v2 <= voltage <= v1:
                return d1 + (d2 - d1) * (voltage - v1) / (v2 - v1)

        return None  # In case voltage doesn't fit in any range

    def run(self):
        """Read voltage from the sensor and return the corresponding distance."""
        voltage = self.channel.voltage
        distance = self.voltage_to_distance(voltage)
        print("Voltage ", voltage)
        print("Distance", distance)
        return distance


if __name__ == '__main__':
    # Create an instance of the DistanceSensor class
    sensor = DistanceSensor(
        clock_pin=board.SCK,
        miso_pin=board.MISO,
        mosi_pin=board.MOSI,
        cs_pin=board.D12,
        adc_pin=MCP.P0
    )

    # Main loop to continuously get and print distance readings
    while True:
        distance = sensor.run()
        print(f"Distance: {distance:.2f} cm")
        time.sleep(0.5)
