import numpy as np
from ultralytics import  YOLO
from ultralytics.yolo.v8.segment.predict import SegmentationPredictor
import cv2 as cv

original_image = cv.imread(r"E:\Thesis\App_Data\Test_eye_to_hand\test4.jpg")  # Replace with actual path to your image
resize_img = cv.resize(original_image, (640,640), interpolation = cv.INTER_AREA)
model = YOLO("E:/Thesis/App_Data/best.pt")
results = model.predict(show=True, source = resize_img)
cv.waitKey(0)
import torchvision.transforms as T
img = T.ToPILImage()(results[0][0].masks.masks)
binary_image = np.array(img)
binary_image = cv.resize(binary_image,(1700,1200))
contours, hierarchy = cv.findContours(binary_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

# Draw contours on a blank image to visualize them
contour_image = np.zeros_like(binary_image)
cv.drawContours(contour_image, contours, -1, (255, 255, 255), thickness=cv.FILLED)

# Get minimum bounding rectangles for each contour
rectangles = [cv.minAreaRect(cnt) for cnt in contours]

# Draw minimum bounding rectangles on the original image

original_image = cv.resize(original_image,(1700,1200))
for rect in rectangles:
    box = cv.boxPoints(rect)
    box = np.int0(box)
    cv.drawContours(original_image, [box], 0, (0, 0, 255), thickness=2)

# Display the final image
cv.namedWindow('Original Image with Minimum Bounding Rectangles', cv.WINDOW_NORMAL)
cv.resizeWindow('Original Image with Minimum Bounding Rectangles', 640,640)
cv.imshow("Original Image with Minimum Bounding Rectangles", original_image)
cv.waitKey(0)
cv.destroyAllWindows()
# rect = cv.minAreaRect(img)
# box = cv.boxPoints(rect)
# box = np.int0(box)
# cv.imshow('result',box)
# cv.waitKey(0)
import cv2


# def LaserCenter(img):
#     rows, cols = img.shape
#     center = np.zeros((rows, cols))
#     # find the center point
#     for x in range(cols):
#         sum1 = 0.0
#         sum2 = 0.0
#         roi = np.where(img[:, x] == 255)
#         if roi[0].size != 0:
#             for y in roi[0]:
#                 sum1 += y * img[y][x]
#                 sum2 += img[y][x]
#             center[int(sum1 / sum2)][x] = 255
#     return center
#
# """ test get laser line"""
# import cv2
# import numpy as np
# img = cv2.imread(r"E:\New_Code\App_Data\Scan_data2\weldseam_85.jpg")
#
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# blur = cv2.GaussianBlur(gray, (7, 7), 0)  # use when img is gray image
# _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)
# cv2.namedWindow('thresh', cv2.WINDOW_NORMAL)
# cv2.resizeWindow('thresh', 640,640)
# cv2.imshow("thresh",thresh)
# cv2.waitKey(0)
# # apply Canny edge detection to identify the laser line
# # edges = cv2.Canny(thresh, 150, 200)
#
# # find contours in the edges image
# # contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# # img1 = cv2.drawContours(gray, contours, -1, (255, 255, 255), 2)
# # img_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
# # cv2.imshow('image', img)
# # cv2.waitKey(0)
# # thinning the contours
# thinned_contours = cv2.ximgproc.thinning(thresh)
# line = LaserCenter(thinned_contours)
# rows , cols = thinned_contours.shape
# points = []
# for i in range(500, rows - 200, 1):
#     for j in range(600, cols - 900, 1):
#         if line[i][j] == 255:
#             cv2.circle(img, (j,i), 5, [255, 0, 0], 2)
#             points.append((j,i))
# points = np.array(points)
# print(points)
# m, b = np.polyfit(points[:, 0], points[:, 1], 1)
# x1 = 788
# y1 = m * x1 + b
# y1 = int(y1)
# x2 = 784
# y2 = m * x2 + b
# y2 = int(y2)
# point1 = (x1,y1)
# point2 = (x2,y2)
# print (x1,y1)
# cv2.circle(img, point1, 5, [0, 255, 0], 2)
# cv2.circle(img, point2, 5, [0, 255, 0], 2)
# # draw the thinned contours on the original image
#
# # display the image
# cv2.namedWindow('image', cv2.WINDOW_NORMAL)
# cv2.resizeWindow('image', 640,640)
# cv2.imshow('image',img)
# cv2.waitKey(0)