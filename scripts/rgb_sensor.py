import board
import adafruit_tcs34725


class RGB:

    def __init__(self):
        # Create sensor object, communicating over the board's default I2C bus
        i2c = board.I2C()  # uses board.SCL and board.SDA
        # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
        self.sensor = adafruit_tcs34725.TCS34725(i2c)