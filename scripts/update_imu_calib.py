#!/usr/bin/env python3

import sys
from pathlib import Path
import xml.etree.ElementTree as ET


def read_acc_calib(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    matrix = []
    for line in lines:
        row = [float(x) for x in line.strip().split()]
        matrix.append(row)
    return matrix


def read_mag_calib(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    min_vals = [int(x) for x in lines[0].strip().split()[1:]]
    max_vals = [int(x) for x in lines[1].strip().split()[1:]]
    return min_vals, max_vals


def update_launch_file(launch_file, acc_matrix, mag_min, mag_max):
    tree = ET.parse(launch_file)
    root = tree.getroot()
    
    node = root.find('node')
    if node is None:
        print("error: node element not found")
        return False
    
    for param in node.findall('param'):
        name = param.get('name')
        if name is None:
            continue
        
        if name == 'mag_min_x':
            param.set('value', str(mag_min[0]))
        elif name == 'mag_min_y':
            param.set('value', str(mag_min[1]))
        elif name == 'mag_min_z':
            param.set('value', str(mag_min[2]))
        elif name == 'mag_max_x':
            param.set('value', str(mag_max[0]))
        elif name == 'mag_max_y':
            param.set('value', str(mag_max[1]))
        elif name == 'mag_max_z':
            param.set('value', str(mag_max[2]))
        elif name.startswith('acc_cali_'):
            row = int(name[-2]) - 1
            col = int(name[-1]) - 1
            param.set('value', str(acc_matrix[row][col]))
    
    tree.write(launch_file, encoding='utf-8', xml_declaration=False)
    return True


def main():
    ws_root = Path(__file__).parent.parent
    acc_calib_file = ws_root / 'results' / 'acc_calib.txt'
    mag_calib_file = ws_root / 'results' / 'mag_calib.txt'
    launch_file = ws_root / 'src' / 'zumo' / 'zumo_launch' / 'launch' / 'zumo_imu_kf.launch'
    
    if not acc_calib_file.exists():
        print(f"error: {acc_calib_file} not found")
        sys.exit(1)
    
    if not mag_calib_file.exists():
        print(f"error: {mag_calib_file} not found")
        sys.exit(1)
    
    acc_matrix = read_acc_calib(acc_calib_file)
    mag_min, mag_max = read_mag_calib(mag_calib_file)
    
    if update_launch_file(launch_file, acc_matrix, mag_min, mag_max):
        print(f"updated {launch_file}")
    else:
        print("failed to update launch file")
        sys.exit(1)


if __name__ == '__main__':
    main()
