import cv2
import numpy as np

class Square:
    """
    Class holding the position, index, corners, empty colour and state of a chess square
    """
    def __init__(self, position, c1, c2, c3, c4, index, image, state=''):
        # ID
        self.position = position
        self.index = index
        # Corners
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3
        self.c4 = c4
        # State
        self.state = state

        # Actual polygon as a numpy array of corners
        self.contours = np.array([c1, c2, c3, c4], dtype=np.int32)

        # Properties of the contour
        self.area = cv2.contourArea(self.contours)
        self.perimeter = cv2.arcLength(self.contours, True)

        M = cv2.moments(self.contours)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # ROI is the small circle within the square on which we will do the averaging
        self.roi = (cx, cy)
        self.radius = 5

        # Empty color. The colour the square has when it's not occupied, i.e. shade of black or white. By storing these
        # at the beginnig of the game, we can then make much more robust predictions on how the state of the board has
        # changed.
        self.emptyColor = self.roiColor(image)