
# Refactored core components with scan listbox selection and custom scan name support
# Auto-selects next scan in the list after completion

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
import pygame
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-D", "--dir", help="Directory to save log files", required=True)
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
    buffer.append(",".join(map(str, row)) + "\n")
    if len(buffer) >= max_size:
        file.writelines(buffer)
        file.flush()
        buffer.clear()

def serial_logger(pico_path, port="/dev/ttyACM0", baudrate=115200):
    global shared_start_time
    base_pico_time = None
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            ser.write(b"RESET\n")
    except Exception as e:
        print("Failed to send reset command:", e)
    time.sleep(0.1)
    pico_buffer = []
    # Regex is straight from ChatGPT... grabs output from rp2040 main.py
    pattern = re.compile(r"(\d+),(-?\d+),([-+]?\d*\.\d+|None),([-+]?\d*\.\d+|None),([-+]?\d*\.\d+|None)")
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        with open(pico_path, 'a') as f_pico:
            while not stop_event.is_set():
                line = ser.readline().decode().strip()
                m = pattern.match(line)
                if not m:
                    continue
                pico_time = float(m.group(1)) / 1e6
                if base_pico_time is None:
                    base_pico_time = pico_time
                    base_pi_time = now()
                    with shared_start_lock:
                        if shared_start_time is None:
                            shared_start_time = base_pi_time
                corrected_time = (pico_time - base_pico_time) + (base_pi_time - shared_start_time)
                _, count, h, r, p = m.groups()
                count = int(count)
                h = float(h) if h != "None" else 0.0
                r = float(r) if r != "None" else 0.0
                p = float(p) if p != "None" else 0.0
                with encoder_val.get_lock():
                    encoder_val.value = count
                pico_data = [corrected_time, h, r, p, count]
                log_line_buffered(pico_buffer, pico_data, f_pico, max_size=1)
    except Exception as e:
        print("Serial logger error:", e)

def extract_lidar_data(sd):
    mean_time = (sd["TimestampStart"] + sd["TimestampStop"]) / 2 / 1e6
    return [sd["Phi"], sd["ChannelTheta"],
            sd["Distance"][0], sd["Rssi"][0],
            mean_time]

def lidar_logger(file_path):
    global shared_start_time
    base_lidar_ts = None
    transport = UDPHandler(IP, PORT, BUFFER)
    receiver = MSGPACKApi.Receiver(transport)
    lidar_buffer = []

    with open(file_path, 'a') as f:
        while not stop_event.is_set():
            segment, _, _ = receiver.receive_segments(1)
            segment = segment[0]
            phi_band = segment["SegmentData"]
            for scan_data in phi_band:
                phi, theta, dist, rssi, lidar_ts = extract_lidar_data(scan_data)
                if base_lidar_ts is None:
                    base_lidar_ts = lidar_ts
                    base_pi_time = now()
                    with shared_start_lock:
                        if shared_start_time is None:
                            shared_start_time = base_pi_time
                corrected_time = (lidar_ts - base_lidar_ts) + (base_pi_time - shared_start_time)
                for i in range(len(theta)):
                    lidar_data = [corrected_time, phi, theta[i], dist[i], rssi[i]]
                    log_line_buffered(lidar_buffer, lidar_data, f, max_size=50)

def get_scan_name():
    global custom_names_used
    if custom_names_used < len(scan_names):
        name = scan_names[custom_names_used]
        custom_names_used += 1
        return name
    idx = 1
    while True:
        name = f"scan_{idx:03d}"
        if not os.path.exists(os.path.join(log_dir, f"{name}_pico.csv")):
            return name
        idx += 1

def start_all_threads(base_name):
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
    stop_event.set()
    for t in threads:
        t.join()

def load_scan_names():
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
    if running:
        with encoder_val.get_lock():
            label.config(text=f"Counter: {encoder_val.value}")
    root.after(100, update_label)

def toggle_logging():
    global running, selected_index, shared_start_time
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
