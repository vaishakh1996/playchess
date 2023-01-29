#!/root/venv/pcd/bin/python3
# Processing of chessboard for opponent move detection

# Rospy for the subscriber
import rospy
from sensor_msgs.msg import PointCloud2, PointField
# from tiago_playchess.msg import chess_move
import numpy as np
#import open3d as o3d
import os
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import yaml
import copy
import cv2
import csv

# User defined script
import config as cfg

# # Defines
# # '/root/tiago_public_ws/src/tiago_playchess/pcl_depth_synchro/01/cloud01.ply'
# pointcloud_for_calibration = '/home/silvia/tiago_public_ws/src/tiago_playchess/pcl_depth_synchro/01/cloud01.ply'
# # '/root/tiago_public_ws/src/tiago_playchess/pcl_depth_synchro/17/cloud17.ply'
# imported_pointcloud = '/home/silvia/tiago_public_ws/src/tiago_playchess/pcl_depth_synchro/17/cloud17.ply'
# # '/root/tiago_public_ws/src/tiago_playchess/pcl_depth_synchro/17/rgb17.png'
# imported_rgb_image = '/home/silvia/tiago_public_ws/src/tiago_playchess/pcl_depth_synchro/17/rgb17.png'
# # '/root/tiago_public_ws/src/tiago_playchess/pcl_depth_synchro/17/depth_norm17.png'
# imported_depth_image = '/home/silvia/tiago_public_ws/src/tiago_playchess/pcl_depth_synchro/17/depth_norm17.png'
# # '/root/tiago_public_ws/src/tiago_playchess/pcl_depth_synchro/17/depth17.csv'
# imported_depth_image_csv = '/home/silvia/tiago_public_ws/src/tiago_playchess/pcl_depth_synchro/17/depth17.csv'
# # '/root/tiago_public_ws/src/tiago_playchess/config/simul_config_not_transformed.yaml'
# imported_chessboard_squares = r'/home/silvia/tiago_public_ws/src/tiago_playchess/config/simul_config_not_transformed.yaml'
# # '/root/tiago_public_ws/src/tiago_playchess/config/chessboard_vertices_not_transformed.yaml'
# imported_chessboard_vertices = r'/home/silvia/tiago_public_ws/src/tiago_playchess/config/chessboard_vertices_not_transformed.yaml'
# # '/root/tiago_public_ws/src/tiago_playchess/scripts/config/simulation_config.yaml'
# imported_configurations = r'/home/silvia/tiago_public_ws/src/tiago_playchess/scripts/config/simulation_config.yaml'
print('x')