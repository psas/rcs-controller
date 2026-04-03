import socket

ip = "192.168.137.108"

angle_error = 5 # degrees of error
kp = 0.003
ki = 0.001
kd = 0.001567
# ki = 0.0
# kd = 0.0
target_heading = 190

state = "0"
# state = "1"

setting_string = state
var_add_list = [angle_error, kp, ki, kd, target_heading]
for var in var_add_list:
    setting_string += "," + str(var)
    
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(setting_string.encode(), (ip, 1234))
print(f"Sent {setting_string} to {ip}")