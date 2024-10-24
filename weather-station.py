from PIL import Image, ImageDraw, ImageFont
import time
import board
from adafruit_mpl3115a2 import MPL3115A2
import busio
import digitalio
import adafruit_ssd1306
import adafruit_tcs34725


# Create sensor object, communicating over the board's default I2C bus
# i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a m>
# sensor = adafruit_tcs34725.TCS34725(i2c)

# Initialize I2C bus
ic2_sen = busio.I2C(board.SCL, board.SDA)

# Initialize the sensor
sensor = MPL3115A2(ic2_sen)

# Prepare the display
oled_reset = digitalio.DigitalInOut(board.D4)

# Change constants 
WIDTH = 128
HEIGHT = 64
BORDER = 2
text_padding = 4
# Exüexpriment with another i2c
i2c = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C, reset=oled_reset)

while(True):
# Create the image
	image = Image.new("1", (oled.width, oled.height))

	draw = ImageDraw.Draw(image)
	draw.rectangle((0,0, oled.width,oled.height), outline=255, fill=255)
	draw.rectangle((BORDER,BORDER,oled.width-BORDER-1, oled.height- BORDER -1),outline=0,fill=0,)

	font = ImageFont.load_default()

# Get data
	altitude = sensor.altitude
	pressure = sensor.pressure
	temperature = sensor.temperature

# Turning the output into a string
	altitude_text = "ALTM: "+str(altitude)+ " m"
	pressure_text = "Pa: " + str(pressure) +" hPA"
	temperature = "Temp: "+ str(temperature)+ " C"

# Calculate positions to center the text
	text_x = BORDER + text_padding
	text_y = BORDER + text_padding


	text_color = 255

	bbox = font.getbbox(altitude_text)
	(font_width, font_height) = bbox[2] - bbox[0], bbox[3] - bbox[1]
# Draw the text for altitude, pressure, and temperature
	draw.text((text_x, text_y), altitude_text, fill=text_color, font=font)
	draw.text((text_x, text_y + font_height + 8), pressure_text, fill=text_color, font=font)
	draw.text((text_x, text_y + 2*font_height + 16), temperature, fill=text_color, font=font)

# Display image
	oled.image(image)
	oled.show()
	time.sleep(1)
