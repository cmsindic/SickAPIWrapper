import os
import argparse
import numpy as np
import open3d as o3d
import pandas as pd

parser = argparse.ArgumentParser()
parser.add_argument("-D", "--dir", required=True, nargs='+',
                    help="One or more directories containing 'synced_clouds'")
parser.add_argument("-o", "--out", default="voxel_summary.csv", help="CSV output file")
args = parser.parse_args()

voxel_size = 0.05
summary_rows = []

for dir_path in args.dir:
	in_dir = os.path.join(dir_path, 'synced_clouds')
	if not os.path.isdir(in_dir):
		print(f"[WARN] {in_dir} not found.")
		continue

	for f in os.scandir(in_dir):
		try:
			df = pd.read_csv(f.path)
			df.columns = df.columns.str.strip()

			points = df[['X', 'Y', 'Z']].to_numpy()
			pcd = o3d.geometry.PointCloud()
			pcd.points = o3d.utility.Vector3dVector(points)

			# SOR filtering
			pcd_clean, ind = pcd.remove_statistical_outlier(nb_neighbors=6, std_ratio=2.0)

			# Voxelization
			voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_clean,
			                                                            voxel_size=voxel_size)
			n_voxels = len(voxel_grid.get_voxels())
			volume = n_voxels * (voxel_size ** 3)

			# Height = 95th percentile of Y in filtered point cloud
			y_values = np.asarray(pcd_clean.points)[:, 1]  # Y-axis
			height_95 = np.percentile(y_values, 95)

			summary_rows.append({
				"file": os.path.basename(f.path),
				"volume_m3": volume,
				"height_95pct_m": height_95,
				"num_voxels": n_voxels,
				"dir": dir_path
			})

		except Exception as e:
			print(f"[ERROR] {f.path}: {e}")

# Write summary CSV
summary_df = pd.DataFrame(summary_rows)
summary_df.to_csv(args.out, index=False)
print(f"Saved summary to {args.out}")
