import socket
import asyncio
import keyboard  # To detect keypresses

pressed_keys = set()

# Function to handle receiving messages from the server asynchronously
async def receive_messages(reader):
    while True:
        try:
            data = await reader.read(1024)  # Receive data asynchronously
            if not data:
                print("Server closed the connection.")
                break
            print("\nServer:", data.decode())
        except Exception as e:
            print(f"An error occurred while receiving data: {e}")
            break

# Function to handle sending messages asynchronously when a key is pressed
def on_key_event(event):
    if event.event_type == 'down':
        pressed_keys.add(event.name)  # Add the key to the set
        print(f"Keys currently pressed: {pressed_keys}")  # Print the set of pressed keys
    
    elif event.event_type == 'up':
        pressed_keys.discard(event.name) 

async def send_on_keypress(writer):
    keyboard.hook(on_key_event)
    while True:
        key = keyboard.read_event()  # Detect any key press event
        if key.event_type == 'down':  # Only consider key press, not release
            char = key.name  # Get the character
            if len(char) == 1:  # Only send valid single characters
                writer.write(char.encode())  # Send each character immediately
                await writer.drain()  # Ensure the data is sent
                print(f"You: {char}", end="", flush=True)  # Print the sent character on the same line

# Asynchronous function to set up the client
async def main():
    # Connect to the server
    reader, writer = await asyncio.open_connection('10.98.212.90', 5000)

    # Create and run tasks for sending and receiving messages
    receive_task = asyncio.create_task(receive_messages(reader))
    send_task = asyncio.create_task(send_on_keypress(writer))

    # Wait for both tasks to complete
    await asyncio.gather(receive_task, send_task)

# Entry point of the program
if __name__ == "__main__":
    try:
        asyncio.run(main())  # Start the event loop and run the main function
    except Exception as e:
        print(f"An error occurred: {e}")
