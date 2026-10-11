#!/usr/bin/env python3
"""
Exports an Odometry topic from a ros2 bag into a CSV with the same column
names your original Listing 3 plotting script expects
(%time, field.pose.pose.position.x, field.pose.pose.position.y), so you
can reuse that script (see plot_from_bag_csvs.py) unchanged.

Usage (run once per topic, after recording with `ros2 bag record`):
    python3 bag_to_csv.py <bag_folder> --topic /fake_gps --out gps.csv
    python3 bag_to_csv.py <bag_folder> --topic /kalman_estimate --out kf.csv
    python3 bag_to_csv.py <bag_folder> --topic /odom --out odom.csv

Requires: rclpy + rosbag2_py (both come with a ROS2 desktop install -
run this from a terminal where you've sourced install/setup.bash).
"""
import argparse
import csv

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions


def export(bag_path, topic, out_path):
    # storage_id='' lets rosbag2_py auto-detect the format (sqlite3/mcap)
    # from the bag's metadata.yaml instead of hardcoding one.
    storage_options = StorageOptions(uri=bag_path, storage_id='')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    rows = []
    while reader.has_next():
        topic_name, data, t = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, Odometry)
        rows.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))

    with open(out_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(
            ['%time', 'field.pose.pose.position.x',
             'field.pose.pose.position.y'])
        writer.writerows(rows)

    print(f'Wrote {len(rows)} rows from {topic} to {out_path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', help='Path to the ros2 bag folder')
    parser.add_argument('--topic', required=True,
                         help='Topic to export, e.g. /fake_gps')
    parser.add_argument('--out', required=True,
                         help='Output CSV path, e.g. gps.csv')
    args = parser.parse_args()
    export(args.bag_path, args.topic, args.out)
