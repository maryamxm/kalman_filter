#!/usr/bin/env python3
"""
This is your original Listing 3 plotter code, unchanged - it reads the
gps.csv / kf.csv / odom.csv produced by bag_to_csv.py and reproduces
Figure 1 from your report.

Usage:
    python3 bag_to_csv.py <bag_folder> --topic /fake_gps --out gps.csv
    python3 bag_to_csv.py <bag_folder> --topic /kalman_estimate --out kf.csv
    python3 bag_to_csv.py <bag_folder> --topic /odom --out odom.csv
    python3 plot_from_bag_csvs.py
"""
import pandas as pd
import matplotlib.pyplot as plt

# --- Load the CSVs ---
gps = pd.read_csv("gps.csv")
kf = pd.read_csv("kf.csv")
odom = pd.read_csv("odom.csv")

# --- Convert timestamps to relative seconds ---
gps['time'] = gps['%time'] - gps['%time'][0]
kf['time'] = kf['%time'] - kf['%time'][0]
odom['time'] = odom['%time'] - odom['%time'][0]

# --- Extract X position (or use Y if you prefer) ---
gps_x = gps['field.pose.pose.position.x']
kf_x = kf['field.pose.pose.position.x']
odom_x = odom['field.pose.pose.position.x']

# --- Plot ---
plt.figure(figsize=(10, 6))
plt.scatter(gps['time'], gps_x, c='r', s=20, label='GPS Measurements')
plt.plot(kf['time'], kf_x, 'b-', linewidth=2, label='Kalman Estimate')
plt.plot(odom['time'], odom_x, 'k--', linewidth=2, label='Ground Truth')

plt.xlabel('Time (seconds)')
plt.ylabel('Position X (meters)')
plt.title('Kalman Filter vs GPS vs Ground Truth')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
