#!/user/bin/env python

#Python libs
import numpy as np
import cv2 
import math
import sys
import operator
# user defined classes
from perceptionLineClass import Line, filterClose
# acessing directories
import os

#ROS libs/packs
import rospy
import ros_numpy

#ROS message 
from sensor_msgs.msg import Image

PLAYCHESS_PKG_DIR = '/home/vaishakh/scripts/vaishakh_scripts/Static_images'


class image_processing():
    def __init__(self):
        self.debug = True
    
    #Canny edge function
    def cannyEdgeDetection(self, img):
        '''
        canny edge detection
        1. used in image_analysis befor findContour
        '''
        edges =cv2.Canny(img,
                         50,    # min_threshold: adjusting it helps to distinguish beteween brown boundary and black  square near to it
                         255
                        )

        #DEBUG
        if self.debug:
            cv2.imshow('Canny',edges)
        
        return edges

    # Hough line
    def categoriseLines(self, lines):
        '''
        used to sort lines  in Haugh lines as horizontal and vertical. Then sort the lines based on their respective center (ascending)
        '''
        horizontal = []
        vertical = []
        for i in range(len(lines)):
            if lines[i].category == 'horizontal':
                horizontal.append(lines[i])
            else:
                vertical.append(lines[i])
        

        horizontal = sorted(horizontal, key=operator.attrgetter('centerH'))
        vertical = sorted(vertical, key=operator.attrgetter('centerV'))
        
        return horizontal,vertical
    
    def houghLines(self, edges, img):
        """
        Detects Hough lines
        edge- o/p of canny
        """

        # Detect hough lines
        lines = cv2.HoughLinesP(edges, rho=1, theta=1 * np.pi / 180, threshold=40, minLineLength=100, maxLineGap=50)
        N = lines.shape[0]

        # Draw lines on image
        New = []
        for i in range(N):
            x1 = lines[i][0][0]
            y1 = lines[i][0][1]
            x2 = lines[i][0][2]
            y2 = lines[i][0][3]

            New.append([x1,y1,x2,y2])

        lines = [Line(x1=New[i][0],y1= New[i][1], x2= New[i][2], y2=New[i][3]) for i in range(len(New))]

        # Categorise the lines into horizontal or vertical
        horizontal, vertical = self.categoriseLines(lines)
        # Filter out close lines based to achieve 9

        # STANDARD THRESHOLD SHOULD BE 20
        ver = filterClose(vertical, horizontal=False, threshold=20)
        hor = filterClose(horizontal, horizontal=True, threshold=20)
        
        # DEBUG TO SHOW LINES
        print(len(ver))
        print(len(hor))
        if self.debug:
            debugImg = img.copy()
            self.drawLines(debugImg, ver)
            self.drawLines(debugImg, hor)
            cv2.imshow(" Hough Lines Found", debugImg)
            cv2.waitKey(0)
        
        return hor, ver

    def drawLines(self, img, lines, color=(0,0,255), thickness=2):
        """
        Draws lines. This function was used to debug Hough Lines generation.
        """
        #print("Going to print: ", len(lines))
        for l in lines:
            l.draw(img, color, thickness)
            ## DEBUG
            cv2.imshow('image', img)
    
    # Intersections 
    def findIntersections(self, horizontals,verticals, image):
        '''
        Finds intersections between Hough lines and filters out close points.
        '''
        intersections = []

        # Find the intersection points 
        for horizontal in horizontals:
            for vertical in verticals:
                d = horizontal.dy*vertical.dx-horizontal.dx*vertical.dy
                dx = horizontal.c*vertical.dx-horizontal.dx*vertical.c
                dy=horizontal.dy*vertical.c-horizontal.c*vertical.dy

                if d != 0:
                    x =abs(int(dx/d))
                    y= abs(int(dy/d))
                else:
                    return False

                intersections.append((x,y))
        if self.debug:
            print("")
            print("We have found: " + str(len(intersections)) + " intersections.")
            print("")
            debugImg = image.copy()

            for intersection in intersections:
                cv2.circle(debugImg, intersection, 10, 255, 1)

            cv2.imshow("Intersections Found", debugImg)
            cv2.waitKey(0)
        ### FILTER

        # Filtering intersection points
        minDistance = 10 
        '''
        lowering minDistance can help find corners far from tiago due to perspective distortion '''

        # Only works if you run it several times -- WHY? Very inefficient
        # Now also works if run only once so comment the loop out
        for i in range(4):
            for intersection in intersections:
                for neighbor in intersections:
                    distanceToNeighbour = np.sqrt((intersection[0] - neighbor[0]) ** 2 + (intersection[1] - neighbor[1]) ** 2)
                    # Check that it's not comparing the same ones
                    if distanceToNeighbour < minDistance and intersection != neighbor:
                        intersections.remove(neighbor)

        # We still have duplicates for some reason. We'll now remove these
        filteredIntersections = []
        # Duplicate removal
        seen = set()
        for intersection in intersections:
            # If value has not been encountered yet,
            # ... add it to both list and set.
            if intersection not in seen:
                filteredIntersections.append(intersection)
                seen.add(intersection)
        if self.debug:
            print("")
            print("We have filtered: " + str(len(filteredIntersections)) + " intersections.")
            print("")

            debugImg =image.copy()

            for intersection in filteredIntersections:
                cv2.circle(debugImg, intersection, 10, (0,0,255), 1)
            cv2.imshow("filtered_intersections", debugImg)
            cv2.waitKey(0)

        return filteredIntersections

    #Assign Intersection 
    def assignIntersections(self, image, intersections):
        """
        Takes the filtered intersections and assigns them to a list containing nine sorted lists, each one representing
        one row of sorted corners. The first list for instance contains the nine corners of the first row sorted
        in an ascending fashion. This function necessitates that the chessboard's horizontal lines are exactly
        horizontal on the camera image, for the purposes of row assignment.
        """

        # Corners array / Each list in list represents a row of corners
        corners = [[],[],[],[],[],[],[],[],[]]

        # Sort rows (ascending)
        intersections.sort(key=lambda x: x[1])
        '''
        key = lambda for sorting x wrt the second element here the secon element is y-co-ordinate of the points'''

        # Assign rows first, afterwards it's possible to swap them around within their rows for correct sequence
        row = 0
        rowAssignmentThreshold = 10

        for i in range(1, len(intersections)):
            if intersections[i][1] in range(intersections[i - 1][1] - rowAssignmentThreshold,
                                            intersections[i - 1][1] + rowAssignmentThreshold):
                corners[row].append(intersections[i - 1])
            else:
                corners[row].append(intersections[i - 1])
                row += 1
            # For last corner
            if i == len(intersections) - 1:
                corners[row].append(intersections[i])

        # Sort by x-coordinate within row to get correct sequence
        for row in corners:
            row.sort(key=lambda x: x[0])

        ## DEBUG
        if self.debug:

            cornerCounter = 0

            debugImg = image.copy()
            for row in corners:
                for corner in row:
                    cv2.circle(debugImg, corner, 10, 255, 1)
                    cornerCounter += 1

            cv2.imshow("4 Final Corners", debugImg)
            

            print("")
            print("There are: " + str(cornerCounter) + " corners that were found.")
            print("")


        return corners, image
    
    # Image_preprocessing
    def preprocessing(self, img,kernelG=5):
        '''
        input image -> GaussianBlur -> Gray -> Adaptive Thrusholding
        '''
        img = cv2.GaussianBlur(img,(kernelG,kernelG),0)

        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        adaptiveThresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)

        # Show  images for debug
        if self.debug:
            cv2.imshow('Gray',gray)
            cv2.imshow("Adaptive_Thresholded",adaptiveThresh)
            cv2.waitKey(0)

        return adaptiveThresh

    # Image analysis
    def image_analysis(self, img, preprocessed_img):
        '''
        finding the contours in the chessboard, filter the largest one and masks it
        
        '''
        ### Chessboard counter extration

        # # canny edge detection
        preprocessed_img = self.cannyEdgeDetection(preprocessed_img)
         

        #Find countours 
        
        imgC, contours, hierarchy = cv2.findContours(preprocessed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        #copy image
        imgContours = img.copy()

        for i in range (len(contours)):
            # Area
            area = cv2.contourArea(contours[i])
            #Peremeter
            perimeter = cv2.arcLength(contours[i], True)
            ''' finding largest counter
                1. avoide zero error
                2. largest contour has largest ratio'''
            if i == 0:
                Lratio = 0
            if perimeter > 0:
                ratio = area/perimeter
                if ratio > Lratio:
                    largest =contours[i]
                    Lratio = ratio
                    Lperimeter = perimeter
                    Larea = area
                else:
                    pass
        # DEBUG 
        color = (255,0,0)
        if self.debug:
            cv2.drawContours(imgContours,[largest],-1,color,2)
            
        #approximation factor
        epsilon = 0.1 *Lperimeter

        # approximating a poligon for chess board edges
        chessboardEdge = cv2.approxPolyDP(largest, epsilon,True)

        #DEBUG
        if self.debug:
            cv2.drawContours(imgContours, [chessboardEdge], -1, color, 2)
            cv2.imshow('contours_in_image',imgContours)   
            cv2.waitKey(0) 
        

        # creating ROI
        roi = cv2.polylines(imgContours,[chessboardEdge],True,(0,255,255),thickness=3)
        # showing filtered contour image
        #DEBUG
        if self.debug:
            cv2.imshow('Filtered_contour',imgContours)
            cv2.waitKey(0)
    
        #creating a mask 
        mask = np.zeros((img.shape[0], img.shape[1]),'uint8')* 125

        # copy chess board edges as a filled white polygon
        cv2.fillConvexPoly(mask,chessboardEdge, 255, 1)
        #coping the chess board to mask
        extracted = np.zeros_like(img)
        extracted[mask ==255] = img[mask == 255]
        #making mask green check later
        # extracted[np.where((extracted == [125, 125, 125]).all(axis=2))]= [0, 0, 20]
        # # add same colored line to remove chess board edges
        cv2.polylines(extracted, [chessboardEdge], True, (0, 255, 0), thickness=5)

        #DEBUD
        if self.debug:
        
            #cv2.imshow("1 Masked", extracted)
            cv2.waitKey(0)

        return extracted

    
 
 #MAIN 

if __name__ == "__main__":
    
    img = cv2.imread('/home/vaishakh/scripts/vaishakh_scripts/Static_images/empty_chess_board.png')
    ip = image_processing()
    preprocessed_img = ip.preprocessing(img)
    analysed_img = ip.image_analysis(img,preprocessed_img)
    canny_img = ip.cannyEdgeDetection(analysed_img)
    hor,ver = ip.houghLines(canny_img,img)
    intersections = ip.findIntersections(hor,ver,img)
    corner, image = ip.assignIntersections(img, intersections)


    cv2.imwrite(PLAYCHESS_PKG_DIR + '/test.png', img)
    cv2.imshow('Finalimg',analysed_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()