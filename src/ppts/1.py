#!/usr/bin/env python3

import csv
import glob
import os
import pickle

folder = "/home/ankit/Desktop/Ankit/Test/APDS/src/lidar_dataset"

for pkl_file in glob.glob(os.path.join(folder, "*.pkl")):

       with open(pkl_file, "rb") as f:
              data = pickle.load(f)
       timestamp = data["timestamp"]

       left_scan = data["input"]["left_scan"]
       right_scan = data["input"]["right_scan"]

       result = data["result"]


       print(f"Created: {result}")

print("All pickle files converted.")