#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import argparse
import os
from datetime import datetime
from scipy.spatial.transform import Rotation as R

def read_calib_file(filepath):
    try:
        calib_file = open(filepath)
    except IOError:
        raise SystemExit(f"Could not open calib file: {filepath}")
    
    calib = {}
    for line in calib_file:
        key, content = line.strip().split(":")
        values = [float(v) for v in content.strip().split()]
        pose = np.zeros((4,4))
        
        pose[0, 0:4] = values[0:4]
        pose[1, 0:4] = values[4:8]
        pose[2, 0:4] = values[8:12]
        pose[3, 3] = 1.0

        calib[key] = pose

    calib_file.close()
    
    return calib

def odometry_callback(msg, callback_args):
    file, calibration = callback_args

    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    norm = np.linalg.norm(quaternion)
    if norm == 0:
        quaternion = [0, 0, 0, 1]
    rotation_matrix = R.from_quat(quaternion).as_matrix()
    
    pose = np.eye(4)
    pose[:3, :3] = rotation_matrix
    pose[0, 3] = position.x
    pose[1, 3] = position.y
    pose[2, 3] = position.z 
    
    Tr = calibration["Tr"]
    Tr_inv = np.linalg.inv(Tr)    
    pose = np.matmul(Tr, np.matmul(pose, Tr_inv))
   
    np.savetxt(file, pose[:3, :].reshape(1, 12), fmt=f'%.{9}f')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='write poses')
    parser.add_argument('--topic_name', type=str, default='/odom', help='Odometry topic name to subscribe')
    parser.add_argument('--calib_file_path', type=str, required=True, help='Path to the calibration file')
    parser.add_argument('--output_path', type=str, required=True, help='Path to save odometry data')
    args = parser.parse_args()

    sensor_calib = read_calib_file(args.calib_file_path)

    timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    output_file_path = os.path.join(args.output_path, f"{timestamp}_poses.txt")
    
    try:
        output_file = open(output_file_path, 'w')
    except IOError:
        raise SystemExit(f"Could not open output file: {output_file_path}")
    
    rospy.init_node('odometry_logger')
    rospy.Subscriber(args.topic_name, Odometry, odometry_callback, callback_args=(output_file, sensor_calib))
    rospy.spin()

    output_file.close()
