import socket
import threading

def handle_client(conn, addr):
    print(f"New connection from {addr}")
    while True:
        data = conn.recv(1024).decode()
        if not data:
            print(f"Connection closed by {addr}")
            break
        print(f"From {addr}: {data}")
        conn.send("Message received".encode())  # Optional response from server

    conn.close()

# Set up the server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('10.98.212.51', 5000))
server_socket.listen(5)

print("Server is listening...")

while True:
    conn, addr = server_socket.accept()
    threading.Thread(target=handle_client, args=(conn, addr)).start()




