""" 
Takes raw lidar data, converts to cartesian coordinates,
and splits into plots given by arguments (in feet).
Outputs are saved to {args.dir}/synced_clouds.

Inputs are two files, one for pico/imu and one for 
the lidar. These are coupled on the basis of their
timestamps. 
"""

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

# Constants... convert feet to mm (units of point cloud... for now).
# Note: outputs of this program ultimately use units of m.
feet_to_mm = 304.8
width = args.width * feet_to_mm
split_mm = args.split * feet_to_mm
start_mm = args.start * feet_to_mm

input_dir = args.dir
out_dir = os.path.join(input_dir, "synced_clouds")
os.makedirs(out_dir, exist_ok=True)

# Max allowed time dif for syncing pico and lidar frames.
DELTA_T_MAX = 0.05 #s

# Utility functions
def _to_cartesian(phi, theta, d):
	""" Convert lidar's phi, theta, distance, to xyz.
	"""
	x = d * np.cos(phi) * np.sin(theta)
	y = d * np.cos(phi) * np.cos(theta)
	z = -d * np.sin(phi)
	return np.stack([x, -y, z], axis=-1)  # Flip y, rot function later flips back

def match_pico_times(lidar_T, pico_T):
	""" Get lidar and pico frames with (nearly) matching 
	timestamps.
	"""
	idxs = np.searchsorted(pico_T, lidar_T)
	idxs = np.clip(idxs, 1, len(pico_T) - 1)
	left = np.abs(pico_T[idxs - 1] - lidar_T)
	right = np.abs(pico_T[idxs] - lidar_T)
	return np.where(left < right, idxs - 1, idxs)

def letter_range(start, end):
	""" Gets a range of plot letter names based on scan file 
	start and end letters. 
		Ex: 1_up_a_j_lidar --> start = a, end = j,
		out = [a, b, ... j]
	"""
	if start <= end:
		return [chr(c) for c in range(ord(start), ord(end) + 1)]
	else:
		return [chr(c) for c in range(ord(start), ord(end) - 1, -1)]
	
def inclusive_range(a, b, step=1):
	""" Same as letter range, but for scan files with 
	numerical nomenclature instead of alphabetic.
	"""
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

def load_csv(path, cols, dtypes=None):
	""" Loads a csv... 
	"""
	try:
		return pd.read_csv(path, names=cols, dtype=dtypes, usecols=range(len(cols)))
	except:
		return pd.DataFrame(columns=cols)

def load_files(scan):
	""" Get lidar and pico files for a scan. Returns as numpy arrays.
	"""
	base = os.path.join(input_dir, scan)
	lidar = load_csv(base + "_lidar.csv", ["T", "Phi", "Theta", "D", "RSSI"],
					 {"T": np.float64, "Phi": np.float32, "Theta": np.float32, "D": np.float32, "RSSI": np.uint16})
	pico = load_csv(base + "_pico.csv", ["T", "Heading", "Roll", "Pitch", "Count"],
					{"T": np.float64, "Heading": np.float32, "Roll": np.float32, "Pitch": np.float32, "Count": np.int32})
	return lidar.to_numpy(), pico.to_numpy()

# Scan identification and classification
scans = {f.rsplit("_", 1)[0] for f in os.listdir(input_dir) if f.endswith("_lidar.csv")}
def is_additional(scan):
	""" Additional scans have a format
	a_b_c.csv 
	where x != int. 
	
	Those need to be identified and treated differently.
	"""
	try:
		t = type(int(scan.split('_')[0]))
		if t == int:
			return False
		else:
			return True
	except Exception as e:
		return True


class Plot:
	""" Plot object to contain point cloud,
	z value range, naming, etc. for an output file.
	"""
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
		""" One scan contains two rows at a time. 
		If traveling up the rows, the lower row is on the 
		right. If traveling down, it is on the left. 
		The polarity of x corresponds with L/R.
		Match x to row and plot object accordingly.
		"""
		r1, r2 = row_options
		left, right = x < 0, x >= 0
		if direction == 'down':
			return (left & (self.row == r2)) | (right & (self.row == r1))
		else:
			return (left & (self.row == r1)) | (right & (self.row == r2))

	def write(self):
		# Skip outfiles already written! 
		# Comment out to overwrite.
		if os.path.exists(self.out_path): return
		
		# Skip empty point clouds. Ignores errors! Comment out to debug. 
		if len(self.cloud) == 0: return
		cloud = np.array(self.cloud)
		cloud[:, [0, 1, 2, 4]] /= 1000 # Converts mm to m !!!!!!!
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
	
	# Minimum z value
	z0 = data[:, 2].min()
	if adtnl:
		row1 = f"{scan}_left"
		row2 = f"{scan}_right"
		for i in range(int(args.n_plots)):
			st = z0 + start_mm + i * split_mm
			fi = z0 + start_mm + (i + 1) * split_mm
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
		x_mask = p.row_match(data[:, 0], direction, row_options)
		p.cloud = data[z_mask & x_mask]
		#print(f"Writing data to {p.name}")
		p.write()

	print(f"Scan {scan} completed")
	
# Make a file containing the names of completed scans if it doesn't exist.
# Overwrite control needs to be treated this way because output files 
# have different names than input files so identification of files that have been
# written to cannot easily be done post facto on the basis of name. 

# WHAT THIS MEANS FOR THE USER: 
# If you want to redo scans completely, you need to delete completed_scans.txt
# and clear the output directory, {args.dir}/synced_clouds.
completed_file_path = os.path.join(args.dir, "completed_scans.txt")
if not os.path.exists(completed_file_path):
	with open(completed_file_path, 'x') as f:
		pass

# Process all scans
for scan in sorted(scans):
	# Overwrite handling.
	with open(completed_file_path, 'r') as f:
		completed_scans = {line.strip() for line in f}
	if scan in completed_scans:
		continue
		
	# Main processing.
	process_scan(scan)
	
	# Overwrite handling.
	with open(completed_file_path, 'a') as f:
		f.write(scan + '\n')
		f.write('')
