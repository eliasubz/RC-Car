import socket
import time

from board import SCL, SDA
import board
import busio
import pwmio
from adafruit_motor import motor

def get_motor():
    i2c = busio.I2C(SCL, SDA)


    pwminb1 = pwmio.PWMOut(board.D19)
    pwminb2 = pwmio.PWMOut(board.D26)
    pwmina1 = pwmio.PWMOut(board.D20)
    pwmina2 = pwmio.PWMOut(board.D21)


    motorL= motor.DCMotor(pwmina1, pwmina2)
    motorR = motor.DCMotor(pwminb1, pwminb2)
    
    return motorL, motorR

def server_program():
    host = "10.98.212.51"
    port = 5000  # Port number

    server_socket = socket.socket()  # Get instance of socket
    server_socket.bind((host, port))  # Bind host and port

    # Server can listen for multiple clients (backlog = 2)
    server_socket.listen(2)
    conn, address = server_socket.accept()  # Accept new connection
    print(f"Connection from: {address}")

    mL, mR = get_motor()

    # Mapping commands to car actions
    while True:
        data = conn.recv(1024).decode()  # Receive data from client
        if not data:
            break  # If no data is received, break the connection
        
        print(f"Command received: {data}")

        # Call function to control car's motors
        control_car(data, mL, mR)

    conn.close()  # Close connection when done

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
