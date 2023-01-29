#!/root/venv/pcd/bin/python3
# Processing of the pointcloud over the chessboard

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
import pickle

# My scripts
import useful_functions.config as cfg
import useful_functions.chess_square_detection as sd 

PLAYCHESS_PKG_DIR = '/home/vaishakh/tiago_public_ws/src/playchess'

#Defines
#importing simulation config to get data like player color
imported_configurations =r'/home/vaishakh/tiago_public_ws/src/playchess/scripts/config/simulation_config.yaml'

class MoveDetection():
    def __init__(self):
        self.debug = True

  # Load configurations
        with open(imported_configurations) as file:
            self.config = yaml.load(file, Loader=yaml.Loader)
        self.color = self.config.get('color')
        self.live_chessboard_situation_complete = cfg.live_chessboard_situation_complete
        self.starting_chessboard_situation_complete = cfg.starting_chessboard_situation_complete
        self.pieces_coordinates = cfg.pieces_coordinates
        self.columns_indexes = cfg.columns
        if self.color == 'white':
            self.squares_to_index = cfg.squares_to_index_white
            self.rows_indexes = cfg.rows_white
            self.opposite_color = 'black'
            self.castle_squares = ['a8', 'e8', 'h8',
                                   'king_e8', 'rook_a8', 'rook_h8']
            self.castle_ending_squares = ['d8', 'c8', 'g8', 'f8']
            self.seventh_row = 'row7'
            self.second_row = 'row2'
        else:
            self.squares_to_index = cfg.squares_to_index_black
            self.rows_indexes = cfg.rows_black
            self.opposite_color = 'white'
            self.castle_squares = ['a1', 'e1', 'h1',
                                   'king_e1', 'rook_a1', 'rook_h1']
            self.castle_ending_squares = ['d1', 'c1', 'g1', 'f1']
            self.seventh_row = 'row2'
            self.second_row = 'row7'

    def star_and_end_squares(self):
        """_summary_:
        to know the starting and end position of the opponent move and also the piece
        """
        with open('/home/vaishakh/tiago_public_ws/src/playchess/scripts/vaishakh_scripts/image_processing/pickle/squares_changed.pickle','rb') as file:
            changed_squares= pickle.load(file)
            self.possible_start={}
            self.possible_end={}
            self.possible_end_capture={}
        for i in changed_squares:
            current_square = i['name']
            occupancy = self.live_chessboard_situation_complete[current_square][1]
            piece = self.live_chessboard_situation_complete[current_square][0]
            if occupancy == self.opposite_color and piece != 'none' :
                self.possible_start.update({current_square: self.live_chessboard_situation_complete[current_square]})
            elif occupancy == self.color:
                self.possible_end_capture.update({current_square: self.live_chessboard_situation_complete[current_square]})
            elif occupancy == 'none' :
                self.possible_end.update({current_square: self.live_chessboard_situation_complete[current_square]})
        print('possible starting squares',self.possible_start)
        print('possible_ending_squares',self.possible_end)
        print('possible capture',self.possible_end_capture)
            


        


if __name__ == '__main__':
    md= MoveDetection()
    md.star_and_end_squares()
    