import cv2
import imutils
from useful_functions.homographic_transformation import HOMO_TRANSFOR as ht
import numpy as np
# from image_processing import ImageProcessing as image_processing
from skimage.measure import compare_ssim
import yaml
from useful_functions.homographic_transformation import HOMO_TRANSFOR as ht


class MoveDetection:
    def __init__(self):
        self.debug = True

    def detectSquareChange(self, previous, current, debug=True):
        """
        Take a previous and a current image and returns the squares where a change happened, i.e. a figure has been
        moved from or to.
        """

        debugImg = current.copy()

        # Convert the images to grayscale
        grayA = cv2.cvtColor(previous, cv2.COLOR_BGR2GRAY)
        grayB = cv2.cvtColor(current, cv2.COLOR_BGR2GRAY)

        # Computes the Structural Similarity Index (SSIM) between previous and current
        (score, diff) = compare_ssim(grayA, grayB, full=True)
        diff = (diff * 255).astype("uint8")

        # DEBUG
        # print("SSIM: {}".format(score))

        # Threshold the difference image, followed by finding contours to obtain the regions of the two input images that differ
        thresh = cv2.threshold(
            diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        cnts = cv2.findContours(
            thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        # Loop over the contours to return centres of differences
        centres = []

        for c in cnts:
            # Compute the bounding box and find its centre
            try:
                # Area
                area = cv2.contourArea(c)
                if area > 100:
                    (x, y, w, h) = cv2.boundingRect(c)
                    centre = (int(x + w / 2), int(y + h / 2))
                    centres.append(centre)
                    cv2.circle(debugImg, centre, 3, 255, 2)
                    cv2.rectangle(debugImg, (x, y),
                                  (x + w, y + h), (0, 0, 255), 2)
            except:
                pass

        # DEBUG
        if debug:
            cv2.imshow("Detected Move", debugImg)
            cv2.waitKey(0)

        return centres

    def showImage(self, image, name="image"):
        """
        Shows the image
        """
        # print("Showing image: '%s'" % name)
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.imshow('image', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    m_d = MoveDetection()
    with open(r'/home/vaishakh/tiago_public_ws/src/playchess/scripts/vaishakh_scripts/image_processing/yml/store_file.yaml') as file:

        chessBoardEdgesS_with_out_borders = yaml.load(
            file, Loader=yaml.FullLoader)
    img_previous = cv2.imread(
        '/home/vaishakh/tiago_public_ws/src/playchess/scripts/vaishakh_scripts/image_processing/Static_images/test.png')
    print(chessBoardEdgesS_with_out_borders)
    transformed_chess_board = ht(
        img_previous, chessBoardEdgesS_with_out_borders, True)
    img_previous = transformed_chess_board.transform()
    img_current = cv2.imread(
        '/home/vaishakh/tiago_public_ws/src/playchess/scripts/vaishakh_scripts/image_processing/Static_images/test1.png')
    transformed_chess_board = ht(
        img_current, chessBoardEdgesS_with_out_borders, True)
    img_current = transformed_chess_board.transform()
    m_d.detectSquareChange(img_current, img_previous, True)
