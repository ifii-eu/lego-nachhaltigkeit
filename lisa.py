import cv2  
import numpy as np
import copy
import math


point = (600, 200)
img = cv2.imread("n1.png")
imghsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)




###############Block###################


###############GreenBlock###################
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

green_corner_H_low = 78
green_corner_S_low = 50
green_corner_V_low = 60
green_corner_H_high = 97
green_corner_S_high = 255
green_corner_V_high = 255

lower = np.array([green_corner_H_low,green_corner_S_low,green_corner_V_low])
upper = np.array([green_corner_H_high,green_corner_S_high,green_corner_V_high])
mask = cv2.inRange(imghsv, lower, upper)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Überprüfen, ob der Punkt in einer der Konturen liegt
for contour in contours:
    distance = cv2.pointPolygonTest(contour, point, False)
    if distance > 0:
        print("Punkt liegt inerhalb der Kontur")

###############YellowBlock###################
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

yellow_corner_H_low = 18
yellow_corner_S_low = 80
yellow_corner_V_low = 170
yellow_corner_H_high = 33
yellow_corner_S_high = 205
yellow_corner_V_high = 255

lower = np.array([yellow_corner_H_low,yellow_corner_S_low,yellow_corner_V_low])
upper = np.array([yellow_corner_H_high,yellow_corner_S_high,yellow_corner_V_high])
mask = cv2.inRange(imghsv, lower, upper)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Überprüfen, ob der Punkt in einer der Konturen liegt
for contour in contours:
    distance = cv2.pointPolygonTest(contour, point, False)
    if distance > 0:
        print("Punkt liegt inerhalb der Kontur")

###############RedBlock###################
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

red1_corner_H_low = 0
red1_corner_S_low = 95
red1_corner_V_low = 0
red1_corner_H_high = 6
red1_corner_S_high = 250
red1_corner_V_high = 255

red2_corner_H_low = 146
red2_corner_S_low = 80
red2_corner_V_low = 0
red2_corner_H_high = 179
red2_corner_S_high = 255
red2_corner_V_high = 255

lower = np.array([red1_corner_H_low,red1_corner_S_low,red1_corner_V_low])
upper = np.array([red1_corner_H_high,red1_corner_S_high,red1_corner_V_high])
mask = cv2.inRange(imghsv, lower, upper)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Überprüfen, ob der Punkt in einer der Konturen liegt
for contour in contours:
    distance = cv2.pointPolygonTest(contour, point, False)
    if distance > 0:
        print("Punkt liegt inerhalb der Kontur")

lower = np.array([red2_corner_H_low,red2_corner_S_low,red2_corner_V_low])
upper = np.array([red2_corner_H_high,red2_corner_S_high,red2_corner_V_high])
mask = cv2.inRange(imghsv, lower, upper)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Überprüfen, ob der Punkt in einer der Konturen liegt
for contour in contours:
    distance = cv2.pointPolygonTest(contour, point, False)
    if distance > 0:
        print("Punkt liegt inerhalb der Kontur")

###############OrangeBlock###################
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

orange_corner_H_low = 7
orange_corner_S_low = 106
orange_corner_V_low = 170
orange_corner_H_high = 17
orange_corner_S_high = 255
orange_corner_V_high = 255

lower = np.array([orange_corner_H_low,orange_corner_S_low,orange_corner_V_low])
upper = np.array([orange_corner_H_high,orange_corner_S_high,orange_corner_V_high])
mask = cv2.inRange(imghsv, lower, upper)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Überprüfen, ob der Punkt in einer der Konturen liegt
for contour in contours:
    distance = cv2.pointPolygonTest(contour, point, False)
    if distance > 0:
        print("Punkt liegt inerhalb der Kontur")

###############GreyBlock###################
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

grey_corner_H_low = 102
grey_corner_S_low = 40
grey_corner_V_low = 140
grey_corner_H_high = 113
grey_corner_S_high = 180
grey_corner_V_high = 255

lower = np.array([grey_corner_H_low,grey_corner_S_low,grey_corner_V_low])
upper = np.array([grey_corner_H_high,grey_corner_S_high,grey_corner_V_high])
mask = cv2.inRange(imghsv, lower, upper)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Überprüfen, ob der Punkt in einer der Konturen liegt
for contour in contours:
    distance = cv2.pointPolygonTest(contour, point, False)
    if distance > 0:
        print("Punkt liegt inerhalb der Kontur")

###############BlueBlock###################
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

blue_corner_H_low = 104
blue_corner_S_low = 140
blue_corner_V_low = 0
blue_corner_H_high = 126
blue_corner_S_high = 255
blue_corner_V_high = 255
#das dunkelste blau

lower = np.array([blue_corner_H_low,blue_corner_S_low,blue_corner_V_low])
upper = np.array([blue_corner_H_high,blue_corner_S_high,blue_corner_V_high])
mask = cv2.inRange(imghsv, lower, upper)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Überprüfen, ob der Punkt in einer der Konturen liegt
for contour in contours:
    distance = cv2.pointPolygonTest(contour, point, False)
    if distance > 0:
        print("Punkt liegt inerhalb der Kontur")

###############PinkBlock###################
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

pink_corner_H_low = 117
pink_corner_S_low = 41
pink_corner_V_low = 161
pink_corner_H_high = 155
pink_corner_S_high = 255
pink_corner_V_high = 255

lower = np.array([pink_corner_H_low,pink_corner_S_low,pink_corner_V_low])
upper = np.array([pink_corner_H_high,pink_corner_S_high,pink_corner_V_high])
mask = cv2.inRange(imghsv, lower, upper)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Überprüfen, ob der Punkt in einer der Konturen liegt
for contour in contours:
    distance = cv2.pointPolygonTest(contour, point, False)
    if distance > 0:
        print("Punkt liegt inerhalb der Kontur")

###############LightGreenBlock###################
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

lightgreen_corner_H_low = 33
lightgreen_corner_S_low = 45
lightgreen_corner_V_low = 0
lightgreen_corner_H_high = 68
lightgreen_corner_S_high = 255
lightgreen_corner_V_high = 255

lower = np.array([lightgreen_corner_H_low,lightgreen_corner_S_low,lightgreen_corner_V_low])
upper = np.array([lightgreen_corner_H_high,lightgreen_corner_S_high,lightgreen_corner_V_high])
mask = cv2.inRange(imghsv, lower, upper)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Überprüfen, ob der Punkt in einer der Konturen liegt
for contour in contours:
    distance = cv2.pointPolygonTest(contour, point, False)
    if distance > 0:
        print("Punkt liegt inerhalb der Kontur")