from CVAnalyzer import CVAnalyzer
import cv2


image = cv2.imread("emptyBoardCropped.jpg")
board_image = cv2.imread("emptyBoardCropped.jpg")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

cv_analyzer = CVAnalyzer()
harris = cv_analyzer.get_harris_corners(gray)
sift = cv_analyzer.get_sift_descriptors(gray,harris)
BIH	= cv_analyzer.find_board_image_homography (board_image)

