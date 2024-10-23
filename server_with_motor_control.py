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


def get_motor():
    i2c = busio.I2C(SCL, SDA)
    return i2c, 0

def server_program():
    host = "10.98.212.51"
    port = 5000  # Port number

    server_socket = socket.socket()  # Get instance of socket
    server_socket.bind((host, port))  # Bind host and port

    # Server can listen for multiple clients (backlog = 2)
    server_socket.listen(2)
    conn, address = server_socket.accept()  # Accept new connection
    print(f"Connection from: {address}")

    # Setting up the motors
    motors = Motors()
    data = ''

    # Setting up the infrared sensor
    infra = Infra()

    # Setting up the rgb sensor
    rgb = RGB()

    # starting time
    prev_time = time.time()
    while True:
        # Checking if there is nothing closer than 20 cm
        if infra.run() < 20:
            print("Something came to close initiating slow retreatment")
            lm, rm = motors.get_motor()
            lm.throttle = -0.1
            lm.throttle = -0.1
            time.sleep(0.5)


        ready_to_read, _, _ = select.select([conn], [], [], 0)
    # Try receiving data (non-blocking mode)

        if ready_to_read: 
            new_data = conn.recv(1024).decode()
            if new_data != "" and len(new_data) == 1:
                data = new_data
        # print(f"New Command that is active: {data}")
        if time.time() - prev_time < 0.1:
            drive_car(data, motors)


        # PID controller or other ongoing tasks can run here
        # For example, you can add logic to control the car's behavior
        # based on sensor inputs, time, etc.

        adjust_alignment(rgb,motors)


    conn.close()  # Close connection when done

def adjust_alignment(rgb,motors):
    r,g,b = rgb
    if r > g+b :
        print("We see red GO Right")
        motors.r_motor.throttle = 0

    elif b > r+g:
        print("We see blue Go LEFT")
        motors.l_motor.throttle = 0

    else: 
        print("We see black alllegidly")
        print("red: ", r, " green ", g, " blue: ", b)
    return 0

def drive_car(command, motors):
    global alpha
    max_speed = 1300  # Maximum motor speed
    
    # Adjust alpha values for speed scaling
    if command == 'j':  
        alpha = 0.75  # 75% speed mode
        print(f"Alpha set to {alpha} (75% speed mode)")
    elif command == 'k':  
        alpha = 0.5  # 50% speed mode
        print(f"Alpha set to {alpha} (50% speed mode)")
    elif command == 'l':  
        alpha = 0.25  # 25% speed mode
        print(f"Alpha set to {alpha} (25% speed mode)")
    elif command == 'h':  
        alpha = 1.0  # Reset speed to full
        print(f"Alpha reset to {alpha} (Full-speed mode)")

    # Handle directional controls
    if command == 'w':
        # Move forward (both motors at the same positive speed)
        motors.run(alpha * max_speed, alpha * max_speed)
        # print(f"Moving forward at speed {alpha * max_speed}")

    elif command == 's':
        # Move backward (both motors at the same negative speed)
        motors.run(-alpha * max_speed, -alpha * max_speed)
        # print(f"Moving backward at speed {alpha * max_speed}")

    elif command == 'a':
        # Turn left (right motor moves forward, left motor slows down or stops)
        motors.run(0, alpha * max_speed)  # Left motor stopped, right motor at speed
        # print(f"Turning left at speed {alpha * max_speed}")

    elif command == 'd':
        # Turn right (left motor moves forward, right motor slows down or stops)
        motors.run(alpha * max_speed, 0)  # Left motor at speed, right motor stopped
        print(f"Turning right at speed {alpha * max_speed}")

    elif command == 'x':
        # Opposite turn (left motor moves backward, right motor forward)
        motors.run(-alpha * max_speed, alpha * max_speed)
        print(f"Opposite turn (left backward, right forward) at speed {alpha * max_speed}")
    
    elif command == 'y':
        # Opposite turn (right motor moves backward, left motor forward)
        motors.run(alpha * max_speed, -alpha * max_speed)
        print(f"Opposite turn (left forward, right backward) at speed {alpha * max_speed}")
    
    elif command == 'e':
        # Right motor at max speed, left motor at half speed (curved right)
        motors.run(alpha * max_speed * 0.5, alpha * max_speed)
        print(f"Curving right (left motor half speed, right motor full speed) at {alpha * max_speed}")

    elif command == 'q':
        # Left motor at max speed, right motor at half speed (curved left)
        motors.run(alpha * max_speed, alpha * max_speed * 0.5)
        print(f"Curving left (left motor full speed, right motor half speed) at {alpha * max_speed}")

    elif command == 'r':
        # Stop both motors
        motors.run(0, 0)
        print("Motors stopped")


def control_car(command, motorR, motorL):
    if command == 'w':
        # Forward
        motorR.throttle = 1.0  # Full speed forward right wheel
        motorL.throttle = 1.0  # Full speed forward left wheel
        print("Moving forward")
    elif command == 's':
        # Backward
        motorR.throttle = -1.0  # Full speed backward right wheel
        motorL.throttle = -1.0  # Full speed backward left wheel
        print("Moving backward")
    elif command == 'a':
        # Turn Left (Right wheel moves, left wheel stops)
        motorR.throttle = 0.0  # Full speed right wheel
        motorL.throttle = 1.0  # Stop left wheel
        print("Turning left")
    elif command == 'd':
        # Turn Right (Left wheel moves, right wheel stops)
        motorR.throttle = 1.0  # Stop right wheel
        motorL.throttle = 0.0  # Full speed left wheel
        print("Turning right")
    elif command == 'x':
        # Opposite turn (right forward, left backward)
        motorR.throttle = 1.0  # Full speed forward right wheel
        motorL.throttle = -1.0  # Full speed backward left wheel
        print("Opposite turn (right forward, left backward)")
    elif command == 'y':
        # Opposite turn (left forward, right backward)
        motorR.throttle = -1.0  # Full speed backward right wheel
        motorL.throttle = 1.0  # Full speed forward left wheel
        print("Opposite turn (left forward, right backward)")
    elif command == 'e':
        # Right wheel max, left wheel half
        motorR.throttle = 1.0  # Full speed right wheel
        motorL.throttle = 0.3  # Half speed left wheel
        print("Right wheel max, left wheel half speed")
    elif command == 'q':
        # Left wheel max, right wheel half
        motorR.throttle = 0.3  # Half speed right wheel
        motorL.throttle = 1.0  # Full speed left wheel
        print("Left wheel max, right wheel half speed")
    elif command == 'r':
        motorR.throttle = 0.0  # Half speed right wheel
        motorL.throttle = 0.0  # Full speed left wheel
        print("Left wheel max, right wheel half speed")
    else:
        print(f"Unknown command: {command}")

if __name__ == '__main__':
    server_program()