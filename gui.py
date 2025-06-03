
"""

A GUI-based multi-threaded data logging utility for synchronized capture of
LIDAR and IMU (RP2040 Pico) data during scan sessions.

Features:
- Supports custom scan filenames via CSV input, and auto-generated names when exhausted
- Synchronizes LIDAR and IMU data streams using a common Pi time reference
- Buffers data for efficient file writing to CSV
- GUI allows user to start/stop scans, view encoder counts, and track output filenames
- Automatically advances to the next scan name upon completion
- Optional audio cues for start/stop events

Usage:
    python scan_logger_gui.py -D <log_directory> [-F <scan_filenames_csv>]

Arguments:
    -D, --dir         Directory where log files will be saved (required)
    -F, --filenames   CSV file with a list of custom scan names (default: scan_filenames.csv)

Dependencies:
    - Python standard libraries: threading, serial, re, time, os, csv, argparse
    - Third-party libraries: pygame, tkinter
    - Custom module: scansegmentapi (UDPHandler and msgpack receiver)

Logging:
- IMU data saved as <scan_name>_pico.csv
- LIDAR data saved as <scan_name>_lidar.csv

Controls (within GUI window):
    - Press 'S' to start/stop a scan
    - Press 'Q' to quit the GUI
    - Use Up/Down arrow keys to select scan from the list
"""

import threading
import serial
import re
import time
import os
import csv
import tkinter as tk
import tkinter.font as tkFont
from tkinter import Listbox, Scrollbar
from scansegmentapi.udp_handler import UDPHandler
import scansegmentapi.msgpack as MSGPACKApi
from multiprocessing import Value
from datetime import date
import pygame
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-D", "--dir", help="Directory to save log files", default=str(date.today()).replace('-','_'))
parser.add_argument("-F", "--filenames", help="File containing scan names", default="scan_filenames.csv")
args = parser.parse_args()
log_dir = args.dir
os.makedirs(log_dir, exist_ok=True)

pygame.mixer.init()

IP = "192.168.0.11"
PORT = 2115
BUFFER = 65535

encoder_val = Value("i", 0)
stop_event = threading.Event()
threads = []

shared_start_time = None
shared_start_lock = threading.Lock()

scan_names = []
custom_names_used = 0
selected_index = 0
file_listbox = None
running = False


##################################################

            ##### DATA PROCESSING #####

##################################################


def now():
    return time.time()

def log_line_buffered(buffer, row, file, max_size=50):
	""" Writes lines to outfile using a buffer where 
	max_size number of lines are saved to memory before 
	being written and buffer is flushed.
	"""
    buffer.append(",".join(map(str, row)) + "\n")
    if len(buffer) >= max_size:
        file.writelines(buffer)
        file.flush()
        buffer.clear()

def serial_logger(pico_path, port="/dev/ttyACM0", baudrate=115200):
	""" Grab data from pico containing IMU.
	Done in a seperate thread from lidar data collection.
	"""
	# Time at which either pico or lidar was initialized (whichever comes first)
	# Used to relativize pico and lidar times for time syncing.
    global shared_start_time
    base_pico_time = None
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            ser.write(b"RESET\n")
    except Exception as e:
        print("Failed to send reset command:", e)
    time.sleep(0.1)
	
	# Buffer containg lines to be written.
	# Used in log_line_buffered()
    pico_buffer = []
	
    # Regex is straight from ChatGPT... grabs output from rp2040 main.py
    pattern = re.compile(r"(\d+),(-?\d+),([-+]?\d*\.\d+|None),([-+]?\d*\.\d+|None),([-+]?\d*\.\d+|None)")
	
    try:
		
		# Initialize pico.
        ser = serial.Serial(port, baudrate, timeout=1)
		
		# Setting up to append to output pico file.
        with open(pico_path, 'a') as f_pico:
			
			# Do while not telling program to stop.
            while not stop_event.is_set():
				
				# Look for pico data signature in stream.
                line = ser.readline().decode().strip()
                m = pattern.match(line)
                if not m:
                    continue
					
                pico_time = float(m.group(1)) / 1e6 # Convert pico's us to s
				
				# Essentially, asks if Pico or Lidar started first 
				# and sets a mutual start time (Pi time)
				# if pico starts clocking first.				
                if base_pico_time is None:
                    base_pico_time = pico_time
                    base_pi_time = now()
                    with shared_start_lock:
                        if shared_start_time is None:
                            shared_start_time = base_pi_time
							
				# Get pico time corrected against pi's start time. 
                corrected_time = (pico_time - base_pico_time) + (base_pi_time - shared_start_time)
				
				# "index, encoder clicks, heading, roll, pitch = pico data"
                _, count, h, r, p = m.groups()
                count = int(count)
                h = float(h) if h != "None" else 0.0
                r = float(r) if r != "None" else 0.0
                p = float(p) if p != "None" else 0.0
				
				# Store as an object accessible to other threads.
                with encoder_val.get_lock():
                    encoder_val.value = count
				
				# Write data to pico's outfile.
                pico_data = [corrected_time, h, r, p, count]
                log_line_buffered(pico_buffer, pico_data, f_pico, max_size=1)
				
    except Exception as e:
        print("Serial logger error:", e)

def extract_lidar_data(sd):
	""" Lidar spits out scan start and scan stop times (us) for each scan. 
	This takes the mean and returns it, in seconds, coupled with
	scan's phi, theta, distance, rssi.
	"""
    mean_time = (sd["TimestampStart"] + sd["TimestampStop"]) / 2 / 1e6
    return [sd["Phi"], sd["ChannelTheta"],
            sd["Distance"][0], sd["Rssi"][0],
            mean_time]

def lidar_logger(file_path):
	""" Logs lidar data in a thread parallel to pico data logging above.
	"""
    global shared_start_time
    base_lidar_ts = None
    transport = UDPHandler(IP, PORT, BUFFER)
    receiver = MSGPACKApi.Receiver(transport)
    lidar_buffer = []

	# While appending output to output file...
    with open(file_path, 'a') as f:
		
		# If user hasn't stopped collection...
        while not stop_event.is_set():
			
			# Get scan segment, one at a time.
            segments, _, _ = receiver.receive_segments(1)
            segment = segments[0] # segments is a list of len == 1
			
			# 1 phi band contains several theta values and corresponding pts.
            phi_band = segment["SegmentData"]
            for scan_data in phi_band:
                phi, theta, dist, rssi, lidar_ts = extract_lidar_data(scan_data)
				
				# As with pico function, 
				# asks if Pico or Lidar started first 
				# and sets a mutual start time (Pi time)
				# if LIDAR starts clocking first.	
                if base_lidar_ts is None:
                    base_lidar_ts = lidar_ts
                    base_pi_time = now()
                    with shared_start_lock:
                        if shared_start_time is None:
                            shared_start_time = base_pi_time
							
				# Lidar time corrected against pi's start time. 
                corrected_time = (lidar_ts - base_lidar_ts) + (base_pi_time - shared_start_time)
				
				# Write to outfile.
                for i in range(len(theta)):
                    lidar_data = [corrected_time, phi, theta[i], dist[i], rssi[i]]
                    log_line_buffered(lidar_buffer, lidar_data, f, max_size=50)

def get_scan_name():
	""" Cycles through scan names set by args.filenames.
	If these names run out, starts using generic output file names.
	These matter for output file names and for the listbox object,
	which allows user to see which files they are writing data to.
	"""
    global custom_names_used
	# While still writing to predeterming scan names...
    if custom_names_used < len(scan_names):
        name = scan_names[custom_names_used]
        custom_names_used += 1
        return name
	
	# Otherwise start creating
	# scan_1_pico(and lidar).csv, scan_2_pico(and lidar).csv...
    idx = 1
    while True:
        name = f"scan_{idx:03d}"
        if not os.path.exists(os.path.join(log_dir, f"{name}_pico.csv")):
            return name
        idx += 1

def start_all_threads(base_name):
	""" Primary controller. 
	Starts data recording.
	"""
    stop_event.clear()
    pico_path = os.path.join(log_dir, f"{base_name}_pico.csv")
    lidar_path = os.path.join(log_dir, f"{base_name}_lidar.csv")
    global threads
    threads = [
        threading.Thread(target=serial_logger, args=(pico_path,), daemon=True),
        threading.Thread(target=lidar_logger, args=(lidar_path,), daemon=True)
    ]
    for t in threads:
        t.start()

def stop_all_threads():
	""" Stops scanning in a way that doesn't cause an error.
	"""
    stop_event.set()
    for t in threads:
        t.join()

def load_scan_names():
	# Get predetermined scan filenames from args.filenames.
    if os.path.exists(args.filenames):
        with open(args.filenames, "r") as f:
            reader = csv.reader(f)
            return [row[0] for row in reader if row]
    return []


##################################################

            ##### GUI STUFF #####

##################################################


root = tk.Tk()
root.title("Scan Logger")
root.geometry("1000x700")
font_big = tkFont.Font(family='Helvetica', size=36, weight='bold')
font_med = tkFont.Font(family='Helvetica', size=24)

label = tk.Label(root, text="Counter: 0", font=font_big)
label.pack(pady=20)

file_label = tk.Label(root, text="Not logging.", font=font_med, fg="blue")
file_label.pack(pady=10)

file_listbox = Listbox(root, font=font_med, width=30)
file_listbox.pack(pady=10)

scan_names = load_scan_names()
for name in scan_names:
    file_listbox.insert(tk.END, name)
file_listbox.insert(tk.END, "(Additional Scans)")
file_listbox.select_set(0)

def update_label():
	""" Updates the number of encoder ticks visible to 
	the user in the GUI.
	"""
    if running:
        with encoder_val.get_lock():
            label.config(text=f"Counter: {encoder_val.value}")
    root.after(100, update_label)

def toggle_logging():
	""" The s key starts a scan if one is not running already
	and stops a scan if one is.
	"""
    global running, selected_index, shared_start_time
	
	# Starts scanning if not scanning.
    if not running:
        selected = file_listbox.curselection()
        if selected:
            selected_index = selected[0]
        base = get_scan_name()
        with encoder_val.get_lock():
            encoder_val.value = 0
        start_all_threads(base)
        file_label.config(text=f"Logging {base}_*.csv")
        start_btn.config(text="Stop Logging", bg="red")
        try:
            pygame.mixer.music.load("start.mp3")
            pygame.mixer.music.play()
        except:
            print("Start sound not found.")
        running = True
	
	# Stops scanning if scanning.
    else:
        stop_all_threads()
        shared_start_time = None
        file_label.config(text="Not logging.")
        start_btn.config(text="Start Logging", bg="green")
        try:
            pygame.mixer.music.load("stop.mp3")
            pygame.mixer.music.play()
        except:
            print("Stop sound not found.")
        running = False

        # Auto-select next scan
        if selected_index < file_listbox.size() - 1:
            selected_index += 1
            file_listbox.select_clear(0, tk.END)
            file_listbox.select_set(selected_index)
            file_listbox.activate(selected_index)

def on_key(event):
	""" Keypress event handler.
	"""
    global selected_index
    if event.keysym.lower() == 's':
        toggle_logging()
    elif event.keysym.lower() == 'q':
        root.destroy()
    elif event.keysym == "Up":
        if selected_index > 0:
            selected_index -= 1
            file_listbox.select_clear(0, tk.END)
            file_listbox.select_set(selected_index)
            file_listbox.activate(selected_index)
    elif event.keysym == "Down":
        if selected_index < file_listbox.size() - 1:
            selected_index += 1
            file_listbox.select_clear(0, tk.END)
            file_listbox.select_set(selected_index)
            file_listbox.activate(selected_index)

start_btn = tk.Button(root, text="Start Logging", command=toggle_logging,
                      bg="green", fg="white", font=font_big)
start_btn.pack(pady=20)

update_label()
root.bind('<Key>', on_key)
root.mainloop()
