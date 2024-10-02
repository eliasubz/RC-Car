import adafruit_lsm303dlh_mag
import adafruit_tcs34725
import board
import adafruit_lsm303_accel
import RPi.GPIO as GPIO
import time
import digitalio
import adafruit_ssd1306
from PIL import Image, ImageDraw,ImageFont

# Define a method (function)
def activate_Led(on,led):
  if(on):
    led.value = True
  else:
    led.value = False

def display(color,oled):
  # Display image
  image = Image.new("1", (oled.width, oled.height))

  # Get drawing object to draw on image.
  draw = ImageDraw.Draw(image)
  font = ImageFont.load_default()

  bbox = font.getbbox(color)
  (font_width, font_height) = bbox[2] - bbox[0], bbox[3] - bbox[1]
  draw.text(
    (oled.width // 2 - font_width // 2, oled.height // 2 - font_height // 2),
    color,
    font=font,
    fill=255,
)
  oled.image(image)
  oled.show()

def get_color(sensor):
  color = sensor.color
  color_rgb = sensor.color_rgb_bytes
  return color_rgb

def classify_color(rgb):
    r, g, b = rgb

    # Check which component is the highest
    if r >= g and r >= b:
        return "Red"
    elif g >= r and g >= b:
        return "Green"
    else:
        return "Blue"

def pointing_north(magnetic):
    x, y, z = magnetic

    # Define a threshold for determining "north"
    threshold = 10  # Adjust based on calibration

    # Check if Z-axis value is significantly larger than X and Y
    if abs(z) > threshold and abs(z) > abs(x) and abs(z) > abs(y):
        return 1
    else:
        return 0

# Main function
def main():
    i2c = board.I2C()
    WIDTH = 128
    HEIGHT = 64
    BORDER = 5
    oled_reset = digitalio.DigitalInOut(board.D4)
    led = digitalio.DigitalInOut(board.D25)
    led.direction = digitalio.Direction.OUTPUT

    oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C, reset=oled_reset)
    color_sensor = adafruit_tcs34725.TCS34725(i2c)
    mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
    magnet_val = mag.magnetic

    activate_Led(1,led)
    time.sleep(1.0)
    activate_Led(0, led)

    while(True):
        magnet_val = mag.magnetic
        print(magnet_val)
        activate_Led(pointing_north(magnet_val),led)
        # Display color
        color_rgb = get_color(color_sensor)
        color_name = classify_color(color_rgb)
        display(color_name, oled)


if __name__ == "__main__":
    main()
