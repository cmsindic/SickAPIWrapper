import os
import numpy as np
import pandas as pd
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-D", "--dir", required=True, help="Directory with scan_###_*.csv files")
parser.add_argument("--split", type=float, default=0.5, help="Feet to travel before starting a new scan chunk")
parser.add_argument("--start", type=float, default=2, help="Feet to travel before starting the first chunk")
parser.add_argument("--n_plots", type=int, default=3, help="Number of subplots per scan")
args = parser.parse_args()

feet_to_m = 0.3048
split_m = args.split * feet_to_m
start_m = args.start * feet_to_m

ref_dir = args.dir
obj_dir = os.path.join(ref_dir, 'synced_clouds')
out_dir = os.path.join(ref_dir, 'sub_split')
os.makedirs(out_dir, exist_ok=True)

references = [f.name for f in os.scandir(ref_dir) if f.name.endswith('.csv') and not 'scan' in f.name]
files = [f for f in os.scandir(obj_dir) if f.name.endswith(".csv") and not 'scan' in f.name]

class ParentScan:
	def __init__(self, name):
		name = os.path.splitext(name)[0]
		r, d = name.split('_')[:2]
		self.direction = d
		self.rows = [str(r), str(int(r)+1)]

class SubPlot:
	def __init__(self, row, rng, sub_number, z_bounds):
		self.row = row
		self.sub_number = sub_number
		self.name = f"{row}_{rng}_{sub_number}"
		self.min_z, self.max_z = z_bounds
		self.out_path = os.path.join(out_dir, f"{self.name}.csv")
		self.cloud = []

	def range_match(self, z):
		return (z > self.min_z) & (z < self.max_z)

	def write(self):
		if os.path.exists(self.out_path): return
		if len(self.cloud) == 0: return
		cloud = np.array(self.cloud)
		pd.DataFrame(cloud, columns=["X", "Y", "Z", "RSSI", "E"]).to_csv(self.out_path, index=False)

# Create scan metadata objects
scan_data = [ParentScan(r) for r in references]

for f in files:
	name = os.path.splitext(f.name)[0]
	row, rng = name.split("_")

	# Determine direction from reference scan
	direction = None
	for r in scan_data:
		if row in r.rows:
			direction = r.direction
			break

	if direction is None:
		print(f"Skipping {name}, no matching direction found.")
		continue

	df = pd.read_csv(f.path)
	data = df[["X", "Y", "Z", "RSSI", "E"]].to_numpy()

	subplot_nums = list(range(1, args.n_plots + 1))
	if direction == 'down':
		subplot_nums = subplot_nums[::-1]

	sub_plots = []
	z0 = data[:, 2].min()
	for i, sub_number in enumerate(subplot_nums):
		st = z0 + start_m + i * split_m
		fi = z0 + start_m + (i + 1) * split_m
		p = SubPlot(row, rng, sub_number, z_bounds=[st, fi])
		sub_plots.append(p)

	for p in sub_plots:
		z_mask = p.range_match(data[:, 2])
		p.cloud = data[z_mask]
		p.write()
