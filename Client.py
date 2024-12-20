import os
import socket
import time

import json
import re

# Define the host and port
HOST = '127.0.0.1'  # Localhost
PORT = 32760  

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Function to handle leading zeros 
def parse_json_with_leading_zeros(json_str): 
    def handle_leading_zeros(match): return str(int(match.group(0))) 
    # Replace numbers with leading zeros
    corrected_json = re.sub(r'\b0\d+', handle_leading_zeros, json_str)
    
    return json.loads(corrected_json)

def receive_data():
 # Example Json { "positions" : [206, 207, 202, 191, 191, 203], "speeds" : [300, 300, 300, 300, 300, 300] }
 # PLC is very picky about the json, so it needs to be axactly 89 characters and values need to be 3 digits
    data = s.recv(89 * 20)
    print(f"Received {data.decode()!r}")
    
    lastData = data[-89:]
    return parse_json_with_leading_zeros(lastData.decode())

def connect():
    while True:
        try:
            s.connect((HOST, PORT))
            print("Connection succesfully established")
            return
        except:
            print("Connection failed, attempting again in 5 seconds...")
            time.sleep(5)
            continue