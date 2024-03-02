import socket
import struct
import numpy as np
import pyigtl
from time import sleep
from math import pi
from scipy.optimize import minimize

def receive_data():
    # Define the IP address and port to listen on
    server_ip = "127.0.0.1"
    server_port = 4000

    # Create a UDP socket
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        # Bind the socket to the specified IP and port
        server_socket.bind((server_ip, server_port))
        # print(f"Listening for UDP data on {server_ip}:{server_port}")

        while True:
            # Receive data and the address it was sent from
            data, address = server_socket.recvfrom(4096)

            # Unpack the received binary data using the format provided
            try:
                # Assuming your data contains doubles
                msg_format = f'{len(data)//8}d'
                unpacked_data = struct.unpack(msg_format, data)
                
                # Process the unpacked data as needed
                # print("Unpacked data:", unpacked_data)
            except struct.error as e:
                print(f"Error unpacking data: {e}")
            return unpacked_data


# Send data to Slicer
server = pyigtl.OpenIGTLinkServer(port=18944, local_server=True)

# Run the function to start listening for data (data to be convert to mm *100 and scaled to world)
rate_x = 100*0.27
rate_y = 100*0.29
rate_z = 100*0.37

if __name__ == "__main__":
    
    
    ############################# Registration #############################
    i = 0
    for i in range (0, 20):
        # Get first point
        received_data = receive_data()
        x_reg = received_data[0]*rate_x
        y_reg = received_data[1]*rate_y
        z_reg = received_data[2]*rate_z
        print("[x_reg, y_reg, z_reg] = ", f'[{x_reg:2.3f}, {y_reg:2.3f}, {z_reg:2.3f}]')
        sleep(3)
        
        # Generate ModelToReference transform
        matrix = np.eye(4)
        matrix[0, 3] = x_reg
        matrix[1, 3] = y_reg
        matrix[2, 3] = z_reg
        transform_message = pyigtl.TransformMessage(matrix, device_name="Registration")
           
        # Send message
        server.send_message(transform_message)
        i += 1
        
    
    ############################# Orientation center #############################
    i = 0
    x_temp=[]
    y_temp=[]
    z_temp=[]
    for i in range (0, 3):
        # Get first point
        received_data = receive_data()
        x_cent = received_data[0]*rate_x
        y_cent = received_data[1]*rate_y
        z_cent = received_data[2]*rate_z
        print("[x_cent, y_cent, z_cent] = ", f'[{x_cent:2.3f}, {y_cent:2.3f}, {z_cent:2.3f}]')
        sleep(3)
        
        # Generate ModelToReference transform
        matrix = np.eye(4)
        matrix[0, 3] = x_cent
        matrix[1, 3] = y_cent
        matrix[2, 3] = z_cent
        transform_message = pyigtl.TransformMessage(matrix, device_name="Registration")
        x_temp.append(x_cent)
        y_temp.append(y_cent)
        z_temp.append(z_cent)
        # Send message
        server.send_message(transform_message)
        i += 1      
  
  
    # Min-square calculation
    points = np.array([[x_temp[0], y_temp[0]], [x_temp[1], y_temp[1]], [x_temp[2], y_temp[2]]])
    print("points =", points)

    # Distance function
    def distance_function(params, points):
        a, b, r = params
        distances = np.sum((points[:, 0] - a)**2 + (points[:, 1] - b)**2 - r**2)**2
        return distances

    # Minimize the distance function
    result = minimize(distance_function, x0=[1, 1, 1], args=(points,), method='Nelder-Mead')

    # Coordinates of the circle center and radius
    center_xy = result.x[:2]
    radius_xy = result.x[2]
    
    
    x_o = center_xy[0]
    y_o = center_xy[1]
    z_o = (z_temp[0] + z_temp[1] + z_temp[2])/3
    
    
    print("[x_o, y_o, z_o] = ", f'[{x_o:2.3f}, {y_o:2.3f}, {z_o:2.3f}]')
    
    # Generate CenterOfOrientation transform
    matrix = np.eye(4)
    matrix[0, 3] = x_o
    matrix[1, 3] = y_o
    matrix[2, 3] = z_o
    transform_message = pyigtl.TransformMessage(matrix, device_name="CenterOfOrientation")
        
    # Send message
    server.send_message(transform_message)
    
    
    # Get sensor position
    received_data = receive_data()
    x_sensor = received_data[0]*rate_x
    y_sensor = received_data[1]*rate_y
    z_sensor = received_data[2]*rate_z
    print("[x_sensor, y_sensor, z_sensor] = ", f'[{x_sensor:2.3f}, {y_sensor:2.3f}, {z_sensor:2.3f}]')
    sleep(5)
    theta_sensor = np.arctan2(y_sensor, x_sensor)
        
    while True:
        # Get position
        received_data = receive_data()
        x = received_data[0]*rate_x
        y = received_data[1]*rate_y
        z = received_data[2]*rate_z

        
        # Get rotation
        if x == 0:
            theta = 0
        else:
            thetar = np.arctan2(y, x) - theta_sensor # radian
            thetad = thetar*180/pi# degrees
    
        print("theta =", thetad, "degree")
        
        # Wait for connection
        if not server.is_connected():
            # Wait for client to connect
            sleep(0.1)
            continue    
          
        # Generate ModelToReference transform
        matrix = np.eye(4)
        matrix[0, 0] = np.cos(thetar)
        matrix[0, 2] = -np.sin(thetar)
        matrix[2, 0] = np.sin(thetar)
        matrix[2, 2] = np.cos(thetar)
        print(matrix)
            
        transform_message = pyigtl.TransformMessage(matrix, device_name="ModelToReference")
        
        # Send message
        server.send_message(transform_message)
        
        
           

