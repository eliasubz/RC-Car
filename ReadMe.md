# Raspberry Pi controlled Car (Embedded Systems)
Python modules to control a robot car that is connected to 2 DC motors and a variety of sensors. The car is controlled by a Raspberry Pi Zero 2w and can operate in two modes: 
- __Autonomous Mode__ - Follows a tape on the flor automatically (GIF)
- __Manual Mode__ - Lets you manually control the cars movement

### Main Scripts 
- ``Pi_Car.py`` 
sets up a socket server on the local network and listens to inputs from a client. It contains the scripts to follow the tape, avoid obstacles and be controlled manually.
- ``remote_control.py`` 
is the second important script, which runs on the laptop. It detects key inputs and sends input commands to the 
pi-car using a simple python client server connection.

![PiCar](Meme/PiCar.gif)

## Used HW components are:
1. Raspberry Pi Zero 2W – Main controller
2. DC Toy Motors (130 Size, 4.5–9V DC) – Drive wheels
3. MCP3008 – 8-channel 10-bit ADC (SPI interface for analog sensors)
4. SHARP GP2Y0A02YK0F – Infrared distance sensor
5. MPL3115A2 – I2C barometric pressure / altitude / temperature sensor
6. Adafruit 9-DOF IMU (LSM303) – Accelerometer and magnetometer
7. TCS34725 RGB Color Sensor – For line following
8. LSM303AGR – Additional accelerometer breakout
9. Monochrome 128x64 OLED – Displays sensor readings and mode status

### Features
- Real-time sensor data integration (distance, color, IMU, pressure)
- Tape-following algorithm using RGB sensor
- Remote control via keyboard over TCP/IP
- Modular Python scripts for easy extension
- Includes simple test scripts for each sensor; feel free to use if you have the same sensor.