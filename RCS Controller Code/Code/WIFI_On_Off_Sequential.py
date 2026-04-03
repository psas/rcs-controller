import socket
from time import sleep

ip = "192.168.137.108"

angle_error = 5 # degrees of error
kp = 0.003
ki = 0.001
kd = 0.001567
# ki = 0.0
# kd = 0.0
heading_list = [90, 180, 0]

state = "0"
state = "1"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def create_string(state, var_list):
    ''' create setting string with all relevant variables to send to board '''
    setting_string = state
    for var in var_add_list:
        setting_string += "," + str(var)
    
    return setting_string
    
def send_signal(string):
    ''' send settings to board via UDP'''
    sock.sendto(string.encode(), (ip, 1234))
    print(f"Sent {string} to {ip}")

if __name__ == "__main__":
    
   if (state == "0"): # turn off controller only once
       var_add_list = [angle_error, kp, ki, kd, 0]
       setting_string = create_string(state, var_add_list)
       send_signal(setting_string)
       print("Controller Off")
       
   else: # send signal to board at set intervals with new headings
       print("Controller On")
       for heading in heading_list:
           var_add_list = [angle_error, kp, ki, kd, heading]
           setting_string = create_string(state, var_add_list)
           send_signal(setting_string)
           sleep(5)
           

