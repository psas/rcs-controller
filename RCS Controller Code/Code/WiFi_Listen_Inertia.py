# import socket

# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.bind(("0.0.0.0", 1235))
# sock.settimeout(0.5)  # Timeout every 0.5 seconds so we can check for stop condition

# print("Listening for UDP messages. Press Ctrl+C or type 'exit' to stop.")

# try:
#     while True:
#         try:
#             data, addr = sock.recvfrom(1024)
#             print(f"From {addr}: {data.decode()} rad/s")
#         except socket.timeout:
#             pass  # just loop again

#         # Optional: non-blocking keyboard input check
#         if input("Type 'exit' to stop or just press enter to keep listening: ") == "exit":
#             break

# except KeyboardInterrupt:
#     print("Stopped by user (Ctrl+C).")

# finally:
#     sock.close()
#     print("Socket closed.")

import socket

# Set up socket to receive
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 1235))  # Port should match `laptopPort` from Arduino
print("Listening for incoming UDP messages...")

while True:
    data, addr = sock.recvfrom(1024)
    print(f"Received from {addr}: {data.decode()}")