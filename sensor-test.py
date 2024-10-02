import time
import board
import busio
from adafruit_mpl3115a2 import MPL3115A2

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the sensor
sensor = MPL3115A2(i2c)

# Loop to read sensor data
while True:
    # Read altitude, pressure, and temperature
    altitude = sensor.altitude  # Altitude in meters
    pressure = sensor.pressure  # Pressure in hPa (hectopascals)
    temperature = sensor.temperature  # Temperature in Celsius

    # Print the sensor readings
    print(f"Altitude: {altitude:.2f} m")
    print(f"Pressure: {pressure:.2f} hPa")
    print(f"Temperature: {temperature:.2f} °C")
    print("---------------------------")
    
    # Delay before next reading
    time.sleep(2)
