#!/user/bin/env python2

import cv2
import pickle
import os
from skimage.measure import compare_ssim
#user defined module
from homographic_transformation import HOMO_TRANSFOR as ht
import numpy as np

PLAYCHESS_PKG_DIR = '/home/vaishakh/tiago_public_ws/src/playchess'



class SquareDetection:
    """_summary_: user definded class to determine difference between image befor opponent move and after opponent move 
        a detect_square_change function used
    """
    def __init__(self):
        self.debug = True

    def detect_square_change(self, previous, current,white="top", debug=False):
        """_summary_ compare two images based on Structural Similarity Index (SSIM)

        Args:
            previous (image): camera image before opponent move
            current (image): camera image after opponent move
            color (str): "black" if TIAGo plays as black. Defaults to "white".
            debug (bool, optional): True for debugging. Defaults to False.

        Returns:
            none
            it stores an pickle file with a list containing detected squares with name squares_changed
        """
        # Convert the images to grayscale
        grayA = cv2.cvtColor(previous, cv2.COLOR_BGR2GRAY)
        grayB = cv2.cvtColor(current, cv2.COLOR_BGR2GRAY)

        # Computes the Structural Similarity Index (SSIM) between previous and current
        (score, diff) = compare_ssim(grayA, grayB, full=True)
        diff = (diff * 255).astype("uint8")

        # Threshold the difference image
        thresh = cv2.threshold(
            diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        img = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        img_copy= img.copy()

        #conforming naming of chess board based on starting position of white pieces
        if white =='top':
            a1_pos=0
        else:
            a1_pos=1
        # Initialize list to store square information
        squares = []
        # Draw squares on image
        for i in range(8):
            for j in range(8):
                # Determine square name
                if a1_pos:
                    square_name = chr(ord('a') + i) + str(8-j)
                else:
                    square_name = chr(ord('a') + 7-i) + str(j+1)
                # Coordinates of square corners
                x1, y1 = i*60, j*60
                x2, y2 = (i+1)*60, (j+1)*60
                # Coordinates of square center
                x_center, y_center = (x1+x2)//2, (y1+y2)//2
                radius = 30
                # # Get sub-image of current square , can be used if circular masking fails
                square = thresh[y1:y2, x1:x2]
                #get window size to get total pixel in one square cell
                height1, width1 = square.shape[:2]
                # Get image dimensions
                height, width = img.shape[:2]
                # Create blank image for mask
                mask = np.zeros((height, width), np.uint8)
                # Draw circle on mask
                cv2.circle(mask, (x_center,y_center), radius, (255, 255, 255), -1)
                # Apply mask to image
                masked_img = cv2.bitwise_and(img, img, mask=mask)
                #converting to single channel
                masked_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
                #cv2.imshow("Image", masked_img)
                #cv2.waitKey(0)
                # Count non-zero pixels in masked region
                non_zero_pixels = cv2.countNonZero(masked_img)
                # Calculate percentage of non-zero pixels
                percent_non_zero = (non_zero_pixels *100/ (height1 * width1) )
                # Check if square contains white pixels
                if percent_non_zero > 18:
                    # Draw square and write square name
                    if self.debug:
                        cv2.rectangle(img_copy, (x1, y1), (x2, y2), (0,255,0), 2)
                        cv2.putText(img_copy, square_name, (x1+20, y1+40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                    # Append square information to list
                    squares.append({"name": square_name, "center": (x_center, y_center), "corners": ((x1, y1), (x2, y2))})


        # Save above list to pickle file which contains details of cell having changs
        with open(os.path.join(PLAYCHESS_PKG_DIR + '/scripts/vaishakh_scripts/image_processing/pickle', 'squares_changed.pickle'), "wb") as f:
            pickle.dump(squares, f)
        
        # DEBUG
        if debug:
            print("detected squares having changes",squares)
            #cv2.imshow("image after SSIM", img)
            cv2.imshow("difference b/w previous and current state of board", img_copy)
            cv2.waitKey(0)

        return 
    
   


if __name__ == '__main__':
    
    m_d = SquareDetection()
    with open(r'/home/vaishakh/tiago_public_ws/src/playchess/scripts/vaishakh_scripts/image_processing/pickle/store_chess_board_edges.pickle', 'rb') as file:
        chessBoardEdgesS_with_out_borders = pickle.load(file)
        #print(chessBoardEdgesS_with_out_borders)
    for i in range(13,14):
        img_path = '/home/vaishakh/tiago_public_ws/src/playchess/scripts/vaishakh_scripts/image_processing/Static_images/Image_move_detection/test{}.png'.format(i)
        img_previous = cv2.imread(img_path)

        img_path = '/home/vaishakh/tiago_public_ws/src/playchess/scripts/vaishakh_scripts/image_processing/Static_images/Image_move_detection/test{}.png'.format(i+1)
        img_current = cv2.imread(img_path)
        
        # img_previous = cv2.imread(
        #      '/home/vaishakh/tiago_public_ws/src/playchess/scripts/vaishakh_scripts/image_processing/Static_images/Image_move_detection/test0.png')
        #print(chessBoardEdgesS_with_out_borders)
        transformed_chess_board = ht(
            img_previous, chessBoardEdgesS_with_out_borders, False)
        img_previous = transformed_chess_board.transform()
        # img_current = cv2.imread(
        #    '/home/vaishakh/tiago_public_ws/src/playchess/scripts/vaishakh_scripts/image_processing/Static_images/Image_move_detection/test1.png')
        transformed_chess_board = ht(
            img_current, chessBoardEdgesS_with_out_borders, False)
        img_current = transformed_chess_board.transform()
        thresh=m_d.detect_square_change(img_current, img_previous,'bottom',True)



'''initial trial and errors
 '''

    # # Read in image
    # img = thresh

    # # Ask user for a1 position
    # # a1_pos = input("Enter 'top'  for a1 position: ")
    # a1_pos =1

    # # Create blank image to draw squares on
    # blank_img = np.zeros((480, 480, 3), np.uint8)
    # img_copy = img.copy()

    # # Initialize list to store square information
    # squares = []

    # # Draw squares on image
    # for i in range(8):
    #     for j in range(8):
    #         # Determine square name
    #         if a1_pos == "top":
    #             square_name = chr(ord('a') + i) + str(8-j)
    #         else:
    #             square_name = chr(ord('a') + 7-i) + str(j+1)
    #         # Coordinates of square corners
    #         x1, y1 = i*60, j*60
    #         x2, y2 = (i+1)*60, (j+1)*60
    #         # # Coordinates of square center
    #         # x_center, y_center = (x1+x2)//2, (y1+y2)//2
    #         # # Get sub-image of current square
    #         # square = thresh[y1:y2, x1:x2]
    #         # # Check if square contains white pixels
    #         # if np.count_nonzero(square) > 0:
    #         #     # Draw square and write square name
    #         #     cv2.rectangle(img_copy, (x1, y1), (x2, y2), (0,255,0), 2)
    #         #     cv2.putText(img_copy, square_name, (x1+20, y1+40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    #         #     # Append square information to list
    #         #     squares.append({"name": square_name, "center": (x_center, y_center), "corners": ((x1, y1), (x2, y2))})

    #         ###########################################################################
    #         # Coordinates of square center
    #         x_center, y_center = (x1+x2)//2, (y1+y2)//2
    #         radius = 30
    #         # Get sub-image of current square
    #         square = thresh[y1:y2, x1:x2]
    #         height1, width1 = square.shape[:2]
    #         # Get image dimensions
    #         height, width = img.shape[:2]
    #         # Create blank image for mask
    #         mask = np.zeros((height, width), np.uint8)
    #         # Draw circle on mask
    #         cv2.circle(mask, (x_center,y_center), radius, (255, 255, 255), -1)
    #         # Apply mask to image
    #         masked_img = cv2.bitwise_and(img, img, mask=mask)
    #         masked_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
    #         #cv2.imshow("Image", masked_img)
    #         #cv2.waitKey(0)
    #         # Count non-zero pixels in masked region
    #         non_zero_pixels = cv2.countNonZero(masked_img)
    #         # Calculate percentage of non-zero pixels
    #         percent_non_zero = (non_zero_pixels *100/ (height1 * width1) )
    #         print((percent_non_zero))
    #         # Check if square contains white pixels
    #         if percent_non_zero > 10:
    #             # Draw square and write square name
    #             cv2.rectangle(img_copy, (x1, y1), (x2, y2), (0,255,0), 2)
    #             cv2.putText(img_copy, square_name, (x1+20, y1+40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    #             # Append square information to list
    #             squares.append({"name": square_name, "center": (x_center, y_center), "corners": ((x1, y1), (x2, y2))})
    #         #########################################################################################
    # # Save list to pickle file
    # with open("squares.pickle", "wb") as f:
    #     pickle.dump(squares, f)

    # # Show image
    # print(squares)
    # cv2.imshow("Image", img_copy)
    # cv2.waitKey(0)