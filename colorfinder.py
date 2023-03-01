#imports
import cv2  
import numpy as np
import copy
import math

#here please change the name of your lego image
# global imgName
# imgName = "foto2.jpg" #e.g. foto2.jpg

def near_point(points, m):
    distances = [math.sqrt((p[0] - m[0])**2 + (p[1] - m[1])**2) for p in points]
    closest_point_index = distances.index(min(distances))
    closest_point = points[closest_point_index]
    return closest_point

def rectangle(points):
    global top_left_point,top_right_point, bottom_left_point, bottom_right_point
    # Die gegebenen Punkte
    # Finden der höchsten und niedrigsten x- und y-Koordinate
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)

    # Identifizieren der Ecken des Rechtecks
    top_left_point = near_point(points, (min_x, min_y))
    top_right_point = near_point(points, (max_x, min_y))
    bottom_left_point = near_point(points, (min_x, max_y))
    bottom_right_point = near_point(points, (max_x, max_y))
    

# Create trackbar callback function
def update_hsv_values(val):
    global corner_H_low, corner_S_low, corner_V_low, corner_H_high, corner_S_high, corner_V_high
    corner_H_low = cv2.getTrackbarPos("corner_H_low", "Trackbars")
    corner_S_low = cv2.getTrackbarPos("corner_S_low", "Trackbars")
    corner_V_low = cv2.getTrackbarPos("corner_V_low", "Trackbars")
    corner_H_high = cv2.getTrackbarPos("corner_H_high", "Trackbars")
    corner_S_high = cv2.getTrackbarPos("corner_S_high", "Trackbars")
    corner_V_high = cv2.getTrackbarPos("corner_V_high", "Trackbars")

def is_point_on_line(point, line_start, line_end, tolerance):
    point_vector = np.array([point[0] - line_start[0], point[1] - line_start[1]])
    line_vector = np.array([line_end[0] - line_start[0], line_end[1] - line_start[1]])
    if np.all(line_vector == 0):
        return False
    return abs(point_vector[0] / line_vector[0] - point_vector[1] / line_vector[1]) < tolerance

def midpoints_calc(points):

	#sortiert nach x Koordinate
	sorted_points = sorted(points, key=lambda x: x[0])
    
    # Calculate the midpoints
	midpoints = []
	new_points = []
	for i in range(len(sorted_points) - 1):
		x1, y1 = sorted_points[i]
		x2, y2 = sorted_points[i+1]
		midpoint = (int((x1+x2)/2), int((y1+y2)/2))
		midpoints.append(midpoint)

	
    
    # Add the points and midpoints to a new list
	for n in range(len(points)):
		new_points.append(points[n])
		if n < len(midpoints):
			new_points.append(midpoints[n])
    
    # Sort the new list
	new_points = sorted(new_points)
	new_points.pop(0)
	new_points.pop(len(new_points)-1)

	return new_points


cap = cv2.VideoCapture(1,cv2.CAP_DSHOW) #video 
k=0
# Create window to show trackbars
cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)

# Create trackbars for H, S and V values
cv2.createTrackbar("corner_H_low", "Trackbars", 0, 179, update_hsv_values)
cv2.createTrackbar("corner_S_low", "Trackbars", 0, 255, update_hsv_values)
cv2.createTrackbar("corner_V_low", "Trackbars", 0, 255, update_hsv_values)
cv2.createTrackbar("corner_H_high", "Trackbars", 179, 179, update_hsv_values)
cv2.createTrackbar("corner_S_high", "Trackbars", 255, 255, update_hsv_values)
cv2.createTrackbar("corner_V_high", "Trackbars", 255, 255, update_hsv_values)

corner_H_low = 66
corner_S_low = 0
corner_V_low = 0
corner_H_high = 99
corner_S_high = 255
corner_V_high = 255
saved_corner_keypoints=[]


while(1):
	# importing time module

	#read source image
	#img=cv2.imread("lego.jpg")
	res, img = cap.read()
	img_green=img
	

	# Convert the image to HSV color space
	rhsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# Define the lower and upper bounds for the red color in the HSV color space
	corner_hsv_low = np.array([corner_H_low, corner_S_low, corner_V_low], np.uint8)
	corner_hsv_high = np.array([corner_H_high, corner_S_high, corner_V_high], np.uint8)

	# Create a mask using the defined lower and upper bounds
	corner_mask = cv2.inRange(rhsv, corner_hsv_low, corner_hsv_high)

	# Perform morphological operations on the mask to remove noise
	kernel = np.ones((5, 5), np.uint8)
	corner_mask = cv2.morphologyEx(corner_mask, cv2.MORPH_OPEN, kernel)
	corner_mask = cv2.morphologyEx(corner_mask, cv2.MORPH_CLOSE, kernel)

	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()
        
	# Change thresholds
	minThreshold = 10  #Default: 10  #ändert nichts?
	maxThreshold = 200  #Default: 200  #ändert nichts?

	# Filter by Area - Nach Fläche filtern
	# This is to avoid any identification of any small dots present in the image that can be wrongly detected as a circle. 
	# Damit sollen kleine Punkte im Bild, die fälschlicherweise als Kreis erkannt werden könnten, nicht erkannt werden.
	filterByArea = True #Default: True  #False ändert sehr viel!!!
	minArea = 0.000001  #Default: 1500 #Better: 50 damit auch noch bei viel Abstand die kleinen Steine erkannt werden! 

	# Filter by Circularity
	# This helps to identify, shapes that are more similar to a circle. 
	# Dies hilft, Formen zu erkennen, die einem Kreis ähnlicher sind.
	filterByCircularity = True  #Default: True  #False ändert nichts
	minCircularity = 0.00001  #Default: 0.1  #Better: 0.0001

	# Filter by Convexity
	# Concavity in general, destroys the circularity. More is the convexity, the closer it is to a close circle. 
	# Konkavität zerstört im Allgemeinen die Kreisform. Je konvexer die Form ist, desto näher ist sie an einem geschlossenen Kreis
	filterByConvexity = True  #Default: True  #False ändert nichts
	minConvexity = 0.00001  #Default: 0.87  #Better: 0.0001

	# Filter by Inertia
	# Objects similar to a circle has larger inertial.E.g. for a circle, this value is 1, for an ellipse it is between 0 and 1, and for a line it is 0. To filter by inertia ratio, set filterByInertia = 1, and set, 0 <= minInertiaRatio <= 1 and maxInertiaRatio (<=1 ) appropriately. 
	# Objekte, die einem Kreis ähneln, haben ein größeres Trägheitsverhältnis, z. B. für einen Kreis ist dieser Wert 1, für eine Ellipse liegt er zwischen 0 und 1 und für eine Linie ist er 0. Um nach dem Trägheitsverhältnis zu filtern, setzen Sie filterByInertia = 1 und setzen Sie 0 <= minInertiaRatio <= 1 und maxInertiaRatio (<=1 ) entsprechend.
	filterByInertia = True  #Default: True  #False ändert nichts
	minInertiaRatio = 0.00001  #Default: 0.01  #Better: 0.0001

	# Create a detector with the parameters
	# OLD_not_working: detector = cv2.SimpleBlobDetector()
	# OLD_working:	detector = cv2.SimpleBlobDetector_create()
	corner_detector = cv2.SimpleBlobDetector_create(params)


	# Apply the mask to the image
	corner_mask = cv2.bitwise_not(corner_mask)

	# Detect blobs.
	corner_mask_keypoints = corner_detector.detect(corner_mask)

	# Draw detected blobs 
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
	corner_keypoints = cv2.drawKeypoints(corner_mask, corner_mask_keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	# Show keypoints
	cv2.imshow("Corner Keypoints", corner_keypoints)

	current_corner_keypoints=[]
	if corner_mask_keypoints:
		for keypoint in corner_mask_keypoints:
			current_corner_keypoints.append(keypoint.pt)
			x, y = keypoint.pt
		
	if(len(current_corner_keypoints)==4):
		saved_corner_keypoints=copy.deepcopy(current_corner_keypoints)
		
	if(len(saved_corner_keypoints)==4):			
		saved_corner_keypoints[0]=(int(saved_corner_keypoints[0][0]),int(saved_corner_keypoints[0][1]))
		saved_corner_keypoints[1]=(int(saved_corner_keypoints[1][0]),int(saved_corner_keypoints[1][1]))
		saved_corner_keypoints[2]=(int(saved_corner_keypoints[2][0]),int(saved_corner_keypoints[2][1]))
		saved_corner_keypoints[3]=(int(saved_corner_keypoints[3][0]),int(saved_corner_keypoints[3][1]))

		rectangle(saved_corner_keypoints)

		cv2.circle(img, bottom_right_point, 5, (100, 200, 200), -1)
		cv2.circle(img, bottom_left_point, 5, (200, 100, 200), -1)
		cv2.circle(img, top_right_point, 5, (200, 200, 100), -1)
		cv2.circle(img, top_left_point, 5, (100, 100, 100), -1)


	#convert sourece image to HSC color mode
	hsv = cv2.cvtColor(img_green, cv2.COLOR_BGR2HSV)

	#Remove to use trackbar !!!
	H_low = 21 #Default: 0 #Better: 21
	H_high = 28 #Default: 179 #Better: 30   
	S_low= 47 #Default: 0 #Better: 47
	S_high = 255 #Default: 255 #Better: 255
	V_low= 100 #Default: 0 #Better: 122 
	V_high = 255 #Default: 255 #Better: 255

	#hsv
	hsv_low = np.array([H_low, S_low, V_low], np.uint8)
	hsv_high = np.array([H_high, S_high, V_high], np.uint8)

	#making mask for hsv range
	mask = cv2.inRange(hsv, hsv_low, hsv_high)
        
	# Führe Rauschentfernung auf der Maske durch
	kernel = np.ones((5,5), np.uint8)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

	#masking HSV value selected color becomes black
	res = cv2.bitwise_and(img, img, mask=mask)      

	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()
        
	# Change thresholds
	params.minThreshold = 10  #Default: 10  #ändert nichts?
	params.maxThreshold = 200  #Default: 200  #ändert nichts?

	# Filter by Area - Nach Fläche filtern
	# This is to avoid any identification of any small dots present in the image that can be wrongly detected as a circle. 
	# Damit sollen kleine Punkte im Bild, die fälschlicherweise als Kreis erkannt werden könnten, nicht erkannt werden.
	params.filterByArea = True #Default: True  #False ändert sehr viel!!!
	params.minArea = 0.000001  #Default: 1500 #Better: 50 damit auch noch bei viel Abstand die kleinen Steine erkannt werden! 

	# Filter by Circularity
	# This helps to identify, shapes that are more similar to a circle. 
	# Dies hilft, Formen zu erkennen, die einem Kreis ähnlicher sind.
	params.filterByCircularity = True  #Default: True  #False ändert nichts
	params.minCircularity = 0.00001  #Default: 0.1  #Better: 0.0001

	# Filter by Convexity
	# Concavity in general, destroys the circularity. More is the convexity, the closer it is to a close circle. 
	# Konkavität zerstört im Allgemeinen die Kreisform. Je konvexer die Form ist, desto näher ist sie an einem geschlossenen Kreis
	params.filterByConvexity = True  #Default: True  #False ändert nichts
	params.minConvexity = 0.00001  #Default: 0.87  #Better: 0.0001

	# Filter by Inertia
	# Objects similar to a circle has larger inertial.E.g. for a circle, this value is 1, for an ellipse it is between 0 and 1, and for a line it is 0. To filter by inertia ratio, set filterByInertia = 1, and set, 0 <= minInertiaRatio <= 1 and maxInertiaRatio (<=1 ) appropriately. 
	# Objekte, die einem Kreis ähneln, haben ein größeres Trägheitsverhältnis, z. B. für einen Kreis ist dieser Wert 1, für eine Ellipse liegt er zwischen 0 und 1 und für eine Linie ist er 0. Um nach dem Trägheitsverhältnis zu filtern, setzen Sie filterByInertia = 1 und setzen Sie 0 <= minInertiaRatio <= 1 und maxInertiaRatio (<=1 ) entsprechend.
	params.filterByInertia = True  #Default: True  #False ändert nichts
	params.minInertiaRatio = 0.00001  #Default: 0.01  #Better: 0.0001

	# Create a detector with the parameters
	# OLD_not_working: detector = cv2.SimpleBlobDetector()
	# OLD_working:	detector = cv2.SimpleBlobDetector_create()
	detector = cv2.SimpleBlobDetector_create(params)

	mask=cv2.bitwise_not(mask)

	# Detect blobs.
	keypoints = detector.detect(mask)


	# Draw detected blobs 
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
	im_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	# Show keypoints
	cv2.imshow("Keypoints", im_with_keypoints)
	#show image mask
	#cv2.imshow('mask',mask)



	# Collect x and y coordinates of keypoints
	x_coords = []
	y_coords = []
	for kp in keypoints:
		x_coords.append(kp.pt[0])
		y_coords.append(kp.pt[1])

	# Calculate the size and position of the yellow rectangle
	if keypoints:
		x_min, x_max = int(min(x_coords)), int(max(x_coords))
		y_min, y_max = int(min(y_coords)), int(max(y_coords))

		# Calculate the center of the yellow rectangle
		x_center = int((x_min + x_max) / 2)
		y_center = int((y_min + y_max) / 2)

		# Calculate the angle between each yellow brick and the center of the yellow rectangle
		angle_dict = {}
		for kp in keypoints:
			x, y = kp.pt
			angle = np.arctan2(y - y_center, x - x_center)
			angle_dict[kp] = angle

		
		# Sort the yellow bricks by their angle
		sorted_bricks = sorted(keypoints, key=lambda kp: angle_dict[kp])

		# Calculate the position of the four corners of the yellow rectangle
		try:
			if(len(saved_corner_keypoints)==4):
				# Assign each brick to a quadrant based on its angle
				left_legos = [bottom_left_point, top_left_point]
				right_legos = [bottom_right_point, top_right_point]
				top_legos = [top_left_point, top_right_point]
				bottom_legos = [bottom_left_point, bottom_right_point]
				for kp in sorted_bricks:
					if(is_point_on_line((int(kp.pt[0]), int(kp.pt[1])),top_left_point,top_right_point,8)):
						top_legos.append((int(kp.pt[0]), int(kp.pt[1])))
					elif(is_point_on_line((int(kp.pt[0]), int(kp.pt[1])),bottom_left_point,bottom_right_point,5)):
						bottom_legos.append((int(kp.pt[0]), int(kp.pt[1])))
					elif(is_point_on_line((int(kp.pt[0]), int(kp.pt[1])),top_left_point,bottom_left_point,5)):
						left_legos.append((int(kp.pt[0]), int(kp.pt[1])))
					elif(is_point_on_line((int(kp.pt[0]), int(kp.pt[1])),top_right_point,bottom_right_point,5)):
						right_legos.append((int(kp.pt[0]), int(kp.pt[1])))		

				# Draw circles at the positions of each Lego brick in the picture
				for kp in keypoints:
					cv2.circle(img, (int(kp.pt[0]), int(kp.pt[1])), 5, (255, 255, 255), -1)

				# Draw circles at the positions of each Lego brick in the picture separated by side
				for position in left_legos:
					cv2.circle(img, position, 3, (0, 0, 255), -1)  # left - red

				for position in right_legos:
					cv2.circle(img, position, 3, (255, 0, 0), -1)  # right - blue

				for position in top_legos:
					cv2.circle(img, position, 3, (255, 255, 0), -1)  # top - cyan

				for position in bottom_legos:
					cv2.circle(img, position, 3, (0, 255, 255), -1)  # bottom - yellow
		except:
			print("error")



  

		# resize image & show result
		scale_percent = 300 # percent of original size
		width = int(img.shape[1] * scale_percent / 100)
		height = int(img.shape[0] * scale_percent / 100)
		dim = (width, height)
		img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
		
		#cv2.imshow('result', img)



	#waitfor the user to press escape and break the while loop 
	k = cv2.waitKey(1) & 0xFF
	if k == 27:
		break
	#//TODO
	if(len(saved_corner_keypoints)==4 and k>20):
		try:
			if(left_legos):
				if(len(left_legos)==len(right_legos) and len(top_legos)==len(bottom_legos)):
					break
		except:
			print("error Left Legos")

	else:
		k+=k



#Sort by Line
right_legos=midpoints_calc(right_legos)
bottom_legos=midpoints_calc(bottom_legos)
top_legos=midpoints_calc(top_legos)
left_legos=midpoints_calc(left_legos)



while(1):
	res, img = cap.read()
	#img=cv2.imread(imgName)
	for kp in keypoints:
		cv2.circle(img, (int(kp.pt[0]), int(kp.pt[1])), 5, (255, 255, 255), -1)

	# Draw circles at the positions of each Lego brick in the picture separated by side
	for position in left_legos:
		cv2.circle(img, position, 2, (0, 0, 255), -1)  # left - red

	for position in right_legos:
		cv2.circle(img, position, 2, (255, 0, 0), -1)  # right - blue

	for position in top_legos:
		cv2.circle(img, position, 2, (255, 255, 0), -1)  # top - cyan

	for position in bottom_legos:
		cv2.circle(img, position, 2, (0, 255, 255), -1)  # bottom - yellow



	# Sortiere die Elemente in right_legos und left_legos nach der y-Koordinate
	right_legos = sorted(right_legos, key=lambda x: x[1])
	left_legos = sorted(left_legos, key=lambda x: x[1])

	# Zeichne Linien zwischen den entsprechenden Elementen in right_legos und left_legos
	for i in range(len(right_legos)):
		x1, y1 = right_legos[i]
		x2, y2 = left_legos[i]
		cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)

	# Zeichne Linien zwischen den entsprechenden Elementen in top_legos und bottom_legos
	for i in range(len(top_legos)):
		x1, y1 = top_legos[i]
		x2, y2 = bottom_legos[i]
		cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)


	intersections = []

	# Draw lines between corresponding elements in right_legos and left_legos
	for i in range(len(right_legos)):
		x1, y1 = right_legos[i]
		x2, y2 = left_legos[i]
		cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)

		# Check if the line from right_legos to left_legos intersects with a line from top_legos to bottom_legos
		for j in range(len(top_legos)):
			x3, y3 = top_legos[j]
			x4, y4 = bottom_legos[j]
			cv2.line(img, (x3, y3), (x4, y4), (0, 255, 0), 2)
			
			# Calculate the intersection point
			denominator = ((x2 - x1) * (y4 - y3)) - ((y2 - y1) * (x4 - x3))
			if denominator == 0:
				continue
			ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denominator
			ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denominator
			if ua >= 0 and ua <= 1 and ub >= 0 and ub <= 1:
				intersection_x = int(x1 + ua * (x2 - x1))
				intersection_y = int(y1 + ua * (y2 - y1))
				intersections.append((intersection_x, intersection_y))



	

	colorfinderarr=[[0]*len(left_legos) for i in range(len(top_legos))]

	counteri=0
	for rangex in range(0,len(left_legos)):
		for rangey in range(0,len(top_legos)):
			colorfinderarr[rangey][rangex]=intersections[counteri]
			counteri+=1




	imgColors=copy.copy(img)
	
	# Convert the image to HSV color space
	hsv = cv2.cvtColor(imgColors, cv2.COLOR_BGR2HSV)

# //TODO 

	# Loop through each intersection point
	for intersection in intersections:
		x, y = intersection

		# Get the HSV values of the pixel at the intersection point
		pixel = hsv[y, x]
		h, s, v = pixel

		# Check the color of the Lego brick based on its hue value
		if h >= 0 and h <= 10 or h >= 350 and h <= 360:
			color = "red"
			code = (0, 0, 255)
		elif h >= 11 and h <= 30:
			color = "orange"
			code = (0, 165, 255)
		elif h >= 31 and h <= 60:
			color = "yellow"
			code = (255, 255, 0)
		elif h >= 61 and h <= 150:
			color = "green"
			code = (0, 255, 0)
		elif h >= 151 and h <= 210:
			color = "cyan"
			code = (0, 255, 255)
		elif h >= 211 and h <= 270:
			color = "blue"
			code = (255, 0, 0)
		elif h >= 271 and h <= 330:
			color = "purple"
			code = (255, 0, 255)
		else:
			color = "pink"
			code = (255, 153, 153)


		# color and position of the Lego brick

		#print("Color:", color, "Position:", x, y)
		cv2.circle(imgColors, (x, y), 4, code, -1)

		cv2.imshow('imgColors', imgColors)






	# Draw circles at the intersection points
	#for intersection in intersections:
	# 	cv2.circle(img, intersection, 5, (0, 0, 255), -1)
	# 	print (intersection)






	# resize image & show result
	#img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
	cv2.imshow('Result', img)


	#waitfor the user to press escape and break the while loop 
	k = cv2.waitKey(1) & 0xFF
	if k == 27:
		break



#destroys all window
cv2.destroyAllWindows()