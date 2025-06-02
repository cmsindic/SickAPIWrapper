import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-D", "--dir", required=True, help="Directory with scan_###_*.csv files")
args = parser.parse_args()

# List of (new name, old name) pairs
rename_pairs = [
    ("40_up_a_j", "a1"),
    ("40_up_k_u", "a3"),
    ("42_down_u_k", "b2"),
    ("42_down_j_a", "scan_001"),
    ("44_up_a_j", "scan_002"),
    ("44_up_k_u", "scan_003"),
    ("46_down_u_k", "scan_004"),
    ("46_down_j_a", "scan_005"),
    ("48_up_a_j", "scan_006"),
    ("48_up_k_u", "scan_007"),
    ("50_down_u_k", "scan_008"),
    ("50_down_j_a", "scan_009"),
    ("52_up_a_j", "scan_010"),
    ("52_up_k_u", "scan_011"),
    ("54_down_u_k", "scan_012"),
    ("54_down_j_a", "scan_013"),
    ("56_up_a_j", "scan_014"),
    ("56_up_k_u", "scan_015"),
    ("58_down_u_k", "scan_016"),
    ("58_down_j_a", "scan_017")
]

folder = args.dir

for new_base, old_base in rename_pairs:
    new_base = new_base.replace(" ", "")  # clean up spaces
    for suffix in ['_pico.csv', '_lidar.csv']:
        old_filename = f'{old_base}{suffix}'
        new_filename = f'{new_base}{suffix}'
        old_path = os.path.join(folder, old_filename)
        new_path = os.path.join(folder, new_filename)

        if os.path.exists(old_path):
            os.rename(old_path, new_path)
            print(f'Renamed: {old_filename} → {new_filename}')
        else:
            print(f'Missing file: {old_filename}')