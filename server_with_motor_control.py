import socket
import time
import select
from board import SCL, SDA
import board
import busio
import pwmio
from adafruit_motor import motor
from motor import Motors
from Infra import Infra
from rgb_sensor import RGB

alpha = 1
r_mov = 0
l_mov = 0
l_th = 0
r_th = 0
change = 0


def get_motor():
    i2c = busio.I2C(SCL, SDA)
    return i2c, 0


def server_program():
    host = "10.98.212.90"
    port = 5000  # Port number

    server_socket = socket.socket()  # Get instance of socket
    server_socket.bind((host, port))  # Bind host and port

    # Server can listen for multiple clients (backlog = 2)
    server_socket.listen(2)
    conn, address = server_socket.accept()  # Accept new connection
    print(f"Connection from: {address}")

    # Setting up the motors
    motors = Motors()
    data = ""

    # Setting up the infrared sensor
    infra = Infra()

    # Setting up the rgb sensor
    rgb = RGB()

    # starting time
    prev_time = time.time()
    forward = 1
    while True:

        #if change % 3 == 0: 
        #    print("Change",change)
        #    old_adjust_alignment(rgb, motors)
        #if change % 3 == 1:
        #    print("Change",change)
        #    adjust_alignment(rgb, motors)
        old_adjust_alignment(rgb, motors)

        ready_to_read, _, _ = select.select([conn], [], [], 0)
        # Try receiving data (non-blocking mode)
        old_adjust_alignment(rgb, motors)
        if ready_to_read:
            new_data = conn.recv(1024).decode()
            print(new_data)
            if new_data != "" and len(new_data) == 1:
                data = new_data

        # print(f" Command that is active: {data}")

        if time.time() - prev_time > 0.1:
            drive_car(data, motors)
            prev_time = time.time()

        # Checking if there is nothing closer than 20 cm

        old_adjust_alignment(rgb, motors)
        forward = adjust_distance(infra, motors, data)#
        

    conn.close()  # Close connection when done


def adjust_distance(infra, motors, data):
    distance = infra.run()
    if distance < 25:
        print("Tell me you are stopping here please")
        # motors.adjust_setpoint(0.5, 0.5)
        motors.l_motor.throttle = -0.17
        motors.r_motor.throttle = -0.17
        while True:
            distance = infra.run()
            if distance > 28:
                motors.l_motor.throttle = 0
                motors.r_motor.throttle = 0
                return 0


    elif distance < 40:
        print("im in the range")
        # if motors.speed_reached():
        # motors.adjust_setpoint(, )
        return 1
        
    return 1


def old_adjust_alignment(rgb, motors):
    r, g, b = rgb.sensor.color_rgb_bytes
    left = motors.l_motor.throttle
    right = motors.r_motor.throttle 
    counter = 0
    if r > g + b:
        print(
            "We see red GO Right",
        )
        motors.r_motor.throttle = 0
        motors.l_motor.throttle = 0.22

        while True:
            r, g, b = rgb.sensor.color_rgb_bytes
            counter += 1
            #if left + right < 0.23:
            if r < g + b:
                if counter < 3:
                    time.sleep(0.05)
                motors.l_motor.throttle = 0.19
                motors.r_motor.throttle = 0.19
                return 0


    elif b > r + g or b > 15:
        print("We see blue Go LEFT")
        motors.l_motor.throttle = 0
        motors.r_motor.throttle = 0.19
        while True:

            r, g, b = rgb.sensor.color_rgb_bytes
            if b < r + g:
                motors.r_motor.throttle += 0.01
                # if left + right < 0.23:
                if counter < 3:
                    time.sleep(0.05)
                motors.l_motor.throttle = 0.19
                motors.r_motor.throttle = 0.19
                return 0
    elif b + r + g > 38:
        motors.l_motor.throttle = -0.17
        motors.r_motor.throttle = -0.17
        time.sleep(1)
    else:
        print("We see black alllegidly")
        print("red: ", r, " green ", g, " blue: ", b)
    return 0


def adjust_alignment(rgb, motors):
    global r_mov
    global l_mov
    global l_th
    global r_th
    r, g, b = rgb.sensor.color_rgb_bytes
    if r > g + b:
        print("We see red GO Right")
        print("red: ", r, " green ", g, " blue: ", b)
        if r_mov == 0:
            r_th = motors.r_motor.throttle
            motors.r_motor.throttle = 0
        throttle = motors.r_motor.throttle
        motors.r_motor.throttle = throttle * 0.1
        r_mov += 1

    elif b > r + g or b > 15:
        print("We see blue Go LEFT")
        print("red: ", r, " green ", g, " blue: ", b)
        if l_mov == 0:
            l_th = motors.l_motor.throttle
            motors.l_motor.throttle = 0
        throttle = motors.l_motor.throttle
        motors.l_motor.throttle = throttle * 0.1
        l_mov +=1

    else:
        if l_mov != 0:
            motors.l_motor.throttle = l_th

        if r_mov != 0:
            motors.r_motor.throttle = r_th
        l_mov = 0
        r_mov = 0

        print("")
        print("We see black alllegidly")
        print("red: ", r, " green ", g, " blue: ", b)
    return 0


def drive_car(command, motors):
    global alpha
    global change 
    max_speed = 1300  # Maximum motor speed

    # Adjust alpha values for speed scaling
    if command == "j":
        alpha = 0.75  # 75% speed mode
        print(f"Alpha set to {alpha} (75% speed mode)")
    elif command == "k":
        alpha = 0.5  # 50% speed mode
        print(f"Alpha set to {alpha} (50% speed mode)")
    elif command == "l":
        alpha = 0.25  # 25% speed mode
        print(f"Alpha set to {alpha} (25% speed mode)")
    elif command == "h":
        alpha = 1.0  # Reset speed to full
        print(f"Alpha reset to {alpha} (Full-speed mode)")

    # Handle directional controls
    if command == "w":
        # Move forward (both motors at the same positive speed)

        motors.run(alpha * max_speed, alpha * max_speed)
        # print(f"Moving forward at speed {alpha * max_speed}")

    elif command == "s":
        # Move backward (both motors at the same negative speed)
        motors.run(-alpha * max_speed, -alpha * max_speed)
        # print(f"Moving backward at speed {alpha * max_speed}")

    elif command == "a":
        # Turn left (right motor moves forward, left motor slows down or stops)
        motors.run(0, alpha * max_speed)  # Left motor stopped, right motor at speed
        # print(f"Turning left at speed {alpha * max_speed}")

    elif command == "d":
        # Turn right (left motor moves forward, right motor slows down or stops)
        motors.run(alpha * max_speed, 0)  # Left motor at speed, right motor stopped
        print(f"Turning right at speed {alpha * max_speed}")

    elif command == "x":
        # Opposite turn (left motor moves backward, right motor forward)
        motors.run(-alpha * max_speed, alpha * max_speed)
        print(
            f"Opposite turn (left backward, right forward) at speed {alpha * max_speed}"
        )

    elif command == "y":
        # Opposite turn (right motor moves backward, left motor forward)
        motors.run(alpha * max_speed, -alpha * max_speed)
        print(
            f"Opposite turn (left forward, right backward) at speed {alpha * max_speed}"
        )

    elif command == "e":
        # Right motor at max speed, left motor at half speed (curved right)
        motors.run(alpha * max_speed * 0.5, alpha * max_speed)
        print(
            f"Curving right (left motor half speed, right motor full speed) at {alpha * max_speed}"
        )

    elif command == "q":
        # Left motor at max speed, right motor at half speed (curved left)
        motors.run(alpha * max_speed, alpha * max_speed * 0.5)
        print(
            f"Curving left (left motor full speed, right motor half speed) at {alpha * max_speed}"
        )
    elif command == "u":
        # Left motor at max speed, right motor at half speed (curved left)
        motors.l_motor.throttle = 1
        motors.r_motor.throttle = 1
        print(
            f"Curving left (left motor full speed, right motor half speed) at {alpha * max_speed}"
        )
    elif command == "p":
            # Left motor at max speed, right motor at half speed (curved left)
            motors.l_motor.throttle = -1
            motors.r_motor.throttle = -1
            print(
                f"Curving left (left motor full speed, right motor half speed) at {alpha * max_speed}"
        )
    
    elif command == "c":
        change += 1
        command = ""


    elif command == "r":
        # Stop both motors
        motors.run(0, 0)
        print("Motors stopped")


if __name__ == "__main__":
    server_program()
