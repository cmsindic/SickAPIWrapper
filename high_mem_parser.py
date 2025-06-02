import os
import sys
import argparse
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

parser = argparse.ArgumentParser()
parser.add_argument("-D", "--dir", required=True, help="Directory with scan_###_*.csv files")
parser.add_argument("--step_mm", type=float, default=0.7752, help="Encoder mm per step")
parser.add_argument("--split", type=float, default=0, help="Feet to travel before starting a new scan chunk (0 = no split)")
parser.add_argument("--start", type=float, default=2, help="Feet to travel before starting the first chunk")
parser.add_argument("--end_buffer", type=float, default=2, help="Feet traveled after ending the scans")
parser.add_argument("--width", type=float, default=10, help="Row width in feet")
parser.add_argument("--n_plots", type=float, default=1, help="Number of plots per scan (only for additional scans)")
args = parser.parse_args()

# Constants
feet_to_mm = 304.8
width = args.width * feet_to_mm
split_mm = args.split * feet_to_mm
start_mm = args.start * feet_to_mm

# Output directory
input_dir = args.dir
out_dir = os.path.join(input_dir, "synced_clouds")
os.makedirs(out_dir, exist_ok=True)

# Max allowed time dif for syncing pico and lidar frames 
DELTA_T_MAX = 0.05 #s

# Utility functions
def _to_cartesian(phi, theta, d):
	x = d * np.cos(phi) * np.sin(theta)
	y = d * np.cos(phi) * np.cos(theta)
	z = -d * np.sin(phi)
	return np.stack([-x, -y, z], axis=-1)  # Flip x, y here, rot flips back

def match_pico_times(lidar_T, pico_T):
	idxs = np.searchsorted(pico_T, lidar_T)
	idxs = np.clip(idxs, 1, len(pico_T) - 1)
	left = np.abs(pico_T[idxs - 1] - lidar_T)
	right = np.abs(pico_T[idxs] - lidar_T)
	return np.where(left < right, idxs - 1, idxs)

def letter_range(start, end):
	return [chr(c) for c in range(ord(start), ord(end) + 1)] if start <= end else [chr(c) for c in range(ord(start), ord(end) - 1, -1)]

def load_csv(path, cols, dtypes=None):
	try:
		return pd.read_csv(path, names=cols, dtype=dtypes, usecols=range(len(cols)))
	except:
		return pd.DataFrame(columns=cols)

def load_files(scan):
	base = os.path.join(input_dir, scan)
	lidar = load_csv(base + "_lidar.csv", ["T", "Phi", "Theta", "D", "RSSI"],
					 {"T": np.float64, "Phi": np.float32, "Theta": np.float32, "D": np.float32, "RSSI": np.uint16})
	pico = load_csv(base + "_pico.csv", ["T", "Heading", "Roll", "Pitch", "Count"],
					{"T": np.float64, "Heading": np.float32, "Roll": np.float32, "Pitch": np.float32, "Count": np.int32})
	return lidar.to_numpy(), pico.to_numpy()

# Scan identification and classification
scans = {f.rsplit("_", 1)[0] for f in os.listdir(input_dir) if f.endswith("_lidar.csv")}
def is_additional(scan):
	""" Additional scans have x_y_z.csv where x != int
	"""
	try:
		t = type(int(scan.split('_')[0]))
		if t == int:
			return False
		else:
			return True
	except Exception as e:
		return True
		

def inclusive_range(a, b, step=1):
	if step == 0:
		raise ValueError("step cannot be zero")
	if a > b and step > 0:
		step = -step
	elif a < b and step < 0:
		step = -step
	# Adjust stop to include the end
	if step > 0:
		return range(a, b + 1, step)
	else:
		return range(a, b - 1, step)

# Plot generation
class Plot:
	def __init__(self, row, letter, b):
		self.row = row
		self.letter = letter
		self.name = f"{row}_{letter}"
		self.min_z, self.max_z = b
		self.out_path = os.path.join(out_dir, f"{self.name}.csv")
		self.cloud = []

	def range_match(self, z):
		return (z > self.min_z) & (z < self.max_z)

	def row_match(self, x, direction, row_options):
		r1, r2 = row_options
		left, right = x < 0, x >= 0
		if direction == 'up':
			return (left & (self.row == r2)) | (right & (self.row == r1))
		else:
			return (left & (self.row == r1)) | (right & (self.row == r2))

	def write(self):
		if os.path.exists(self.out_path): return
		if len(self.cloud) == 0: return
		cloud = np.array(self.cloud)
		cloud[:, [0, 1, 2, 4]] /= 1000
		pd.DataFrame(cloud, columns=["X", "Y", "Z", "RSSI", "E"]).to_csv(self.out_path, index=False)

# Core scan processor
def process_scan(scan):
	print(scan)
	adtnl = is_additional(scan)
	lidar_np, pico_np = load_files(scan)
	if lidar_np.size == 0 or pico_np.size == 0:
		print(f"Skipping scan {scan} due to missing data.")
		return
		
	# Sync lidar and pico frames by time
	lidar_T = lidar_np[:, 0]
	pico_T = pico_np[:, 0]
	idxs = match_pico_times(lidar_T, pico_T)
	valid = np.abs(pico_T[idxs] - lidar_T) <= DELTA_T_MAX
	lidar_np, pico_np = lidar_np[valid], pico_np[idxs[valid]]
	
	# Unpack pico and lidar data
	phi, theta, d = lidar_np[:, 1], lidar_np[:, 2], lidar_np[:, 3]
	rssi, count = lidar_np[:, 4], pico_np[:, 4]
	heading, roll, pitch = pico_np[:, 1], pico_np[:, 2], pico_np[:, 3]
	if heading.shape[0] == 0:
		return
	dist_mm = count * args.step_mm
	
	# Rotate lidar points by imu
	points = _to_cartesian(phi, theta, d)
	rot = R.from_euler('YZX', np.stack([heading, roll, pitch], axis=1), degrees=True)
	world_pts = rot.apply(points)
	world_pts[:, 2] += dist_mm

	# Pack output
	mask = np.abs(world_pts[:, 0]) <= width
	world_pts = world_pts[mask]
	rssi = rssi[mask]
	dist_mm = dist_mm[mask]
	data = np.column_stack([world_pts, rssi, dist_mm])

	# Generate plot objects
	plots = []
	if adtnl:
		row1 = f"{scan}_left"
		row2 = f"{scan}_right"
		for i in range(int(args.n_plots)):
			st = start_mm + i * split_mm
			fi = start_mm + (i + 1) * split_mm
			for row in (row1, row2):
				plots.append(Plot(row, chr(97+i), (st, fi)))
		row_options = [row1, row2]
		direction = 'up'
	else:
		row, direction, a, b = scan.split('_')
		row2 = str(int(row) + 1)
		try:
			lr = letter_range(a, b)
		except:
			lr = inclusive_range(int(a), int(b))
		# When scan is started in the middle,
		# there is no buffer before the plot starts.
		if a == 'j': 
			st_mm = 0
		else:
			st_mm = start_mm
		for i, letter in enumerate(lr):
			st = st_mm + i * split_mm
			fi = st_mm + (i + 1) * split_mm
			plots.append(Plot(row, letter, (st, fi)))
			plots.append(Plot(row2, letter, (st, fi)))
		row_options = [row, row2]
	
	# Assign points to plots and save
	for p in plots:
		z_mask = p.range_match(data[:, 2])
		x_mask = np.ones_like(z_mask, dtype=bool) if adtnl else p.row_match(data[:, 0], direction, row_options)
		p.cloud = data[z_mask & x_mask]
		#print(f"Writing data to {p.name}")
		p.write()

	print(f"Scan {scan} completed")
	

completed_file_path = os.path.join(args.dir, "completed_scans.txt")
if not os.path.exists(completed_file_path):
	with open(completed_file_path, 'x') as f:
		pass

# Process all scans
for scan in sorted(scans):
	with open(completed_file_path, 'r') as f:
		completed_scans = {line.strip() for line in f}
	if scan in completed_scans:
		continue
	process_scan(scan)
	with open(completed_file_path, 'a') as f:
		f.write(scan + '\n')
		f.write('')