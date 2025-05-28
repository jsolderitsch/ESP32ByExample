#! /usr/bin/env python3

import serial
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("ssid", help="supply a Tello SSID here")

args = parser.parse_args()

MAX_BUFF_LEN = 50
SETUP = False
port = None
prev = time.time()

while not SETUP:
    try:
        # Serial port(windows-->COM), baud rate, timeout msg
        # Replace with your Serial port string.
        port = serial.Serial("/dev/cu.usbserial-022AF68B", 115200, timeout=1)
        port.setDTR(True)
        port.setRTS(False)
        port.open()
        # port.flush()

    except:  # Bad way of writing excepts (always know your errors)
        if time.time() - prev > 2:  # Don't spam with msg
            print("No serial detected, please plug your uController")
            prev = time.time()

    if port is not None:  # We're connected
        SETUP = True
        # start with empty serial buffers
        port.reset_input_buffer()
        port.flush()
        port.reset_output_buffer()
        
# read one line
def readline_ser():
    response = port.readline()
    return response.decode()

# Write whole strings
def write_ser(cmd):
    cmd = cmd + '\n'
    port.write(cmd.encode())

# process a  command with an optional delay
# rc commands do not generate a response
def process_tello_command(cmd, delay=0):
    write_ser(cmd)
    for i in range(10):
        command = readline_ser().strip()
        if len(command):
            print(command)
            break
        time.sleep(1)
    if "rc " not in cmd:
        for i in range(20):
            response = readline_ser().strip()
            if len(response):
                print(response)
                break
            time.sleep(1)
    time.sleep(delay)

def process_tello_connect(cmd, delay=2):
    print(cmd)
    write_ser(cmd)
    connected = False
    noRespTime = time.time()
    while not connected:
    	response = readline_ser().strip()
    	if len(response):
    	    print(response)
    	    noRespTime = time.time()
    	time.sleep(delay);
    	if time.time() - noRespTime > 6:
    	    print("Should be connected now")
    	    connected = True

process_tello_connect("connect TELLO-" + args.ssid, 2)

# process sequence of commands with different delays
process_tello_command("motoron", 5)
process_tello_command("motoroff", 0)
process_tello_command("battery?", 0)
process_tello_command("command", 0)

port.close()
