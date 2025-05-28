#! /usr/bin/env python3

import serial
import time

MAX_BUFF_LEN = 50
SETUP = False
port = None
prev = time.time()

while not SETUP:
    try:
        # Serial port(windows-->COM), baud rate, timeout setting
        # Replace with your Serial port string.
        port = serial.Serial("/dev/cu.usbserial-022AF68B", 115200, timeout=1)
        port.setRTS(False)
        port.setDTR(False)
        port.open()

    except:  # Crude way of writing excepts (always know your errors)
        if time.time() - prev > 2:  # Don't spam with msg
            print("No serial detected, please plug your uController")
            prev = time.time()

    if port is not None:  # We're connected
        SETUP = True
        # start with empty buffers
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

# process sequence of commands with different delays
process_tello_command("motoron", 4)
process_tello_command("motoroff", 0)
# process_tello_command("takeoff", 0)
# process_tello_command("up 50", 4)
# process_tello_command("cw 90", 4)
# process_tello_command("ccw 90", 4)
# process_tello_command("down 50", 4)
# process_tello_command("land", 0)
process_tello_command("battery?", 0)
port.close()
