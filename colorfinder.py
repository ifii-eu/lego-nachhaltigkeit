#import opencv and numpy
import cv2  
import numpy as np

cap = cv2.VideoCapture(0,cv2.CAP_DSHOW) #video 

#ad trackbar here

while(1):
	#read source image
	img=cv2.imread("lego.jpg")
	res, img = cap.read()

	#convert sourece image to HSC color mode
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	#Remove to use trackbar !!!
	H_low = 21 #Default: 0 #Better: 21
	H_high = 30 #Default: 179 #Better: 30   
	S_low= 47 #Default: 0 #Better: 47
	S_high = 255 #Default: 255 #Better: 255
	V_low= 122 #Default: 0 #Better: 122 
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
	detector = cv2.SimpleBlobDetector_create(params)

	mask=cv2.bitwise_not(mask)

	# Detect blobs.
	keypoints = detector.detect(mask)


	# Draw detected blobs 
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
	im_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	# Show keypoints
	cv2.imshow("Keypoints", im_with_keypoints)
	#show image mask
	#cv2.imshow('mask',mask)


##########################################################

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

		# //FIXME //TODO ???
		# Sort the yellow bricks by their angle
		sorted_bricks = sorted(keypoints, key=lambda kp: angle_dict[kp])

		# Assign each brick to a quadrant based on its angle
		left_legos = []
		right_legos = []
		top_legos = []
		bottom_legos = []

		# //FIXME //TODO ???
		for kp in sorted_bricks:
			angle = angle_dict[kp]
			if np.pi / 4 <= angle < 3 * np.pi / 4:
				bottom_legos.append((int(kp.pt[0]), int(kp.pt[1])))
			elif -3 * np.pi / 4 <= angle < -np.pi / 4:
				top_legos.append((int(kp.pt[0]), int(kp.pt[1])))
			elif 3 * np.pi / 4 <= angle or angle < -3 * np.pi / 4:
				left_legos.append((int(kp.pt[0]), int(kp.pt[1])))
			else:
				right_legos.append((int(kp.pt[0]), int(kp.pt[1])))


		#print lenght of arrays
		print("left_legos:", len(left_legos))
		print("right_legos:", len(right_legos))
		print("top_legos:", len(top_legos))
		print("left_legos:", len(bottom_legos))	


		# Draw circles at the positions of each Lego brick in the picture
		for kp in keypoints:
			cv2.circle(img, (int(kp.pt[0]), int(kp.pt[1])), 10, (255, 255, 255), -1)

		# Draw circles at the positions of each Lego brick in the picture separated by side
		for position in left_legos:
			cv2.circle(img, position, 10, (0, 0, 255), -1)  # left - red

		for position in right_legos:
			cv2.circle(img, position, 10, (255, 0, 0), -1)  # right - blue

		for position in top_legos:
			cv2.circle(img, position, 10, (255, 255, 0), -1)  # top - cyan

		for position in bottom_legos:
			cv2.circle(img, position, 10, (0, 255, 255), -1)  # bottom - yellow


		# Balance the number of bricks in each quadrant
		min_len = min(len(left_legos), len(right_legos), len(top_legos), len(bottom_legos))
		left_legos = left_legos[:min_len]
		right_legos = right_legos[:min_len]
		top_legos = top_legos[:min_len]
		bottom_legos = bottom_legos[:min_len]
		
		# Connect each brick in one quadrant with a specific brick in the opposite quadrant
		for i in range(min_len):
			cv2.line(img, tuple(left_legos[i]), tuple(right_legos[min_len-i-1]), (0, 255, 0), 2)
			cv2.line(img, tuple(top_legos[i]), tuple(bottom_legos[min_len-i-1]), (0, 255, 0), 2)


		# show result
		scale_percent = 300 # percent of original size
		width = int(img.shape[1] * scale_percent / 100)
		height = int(img.shape[0] * scale_percent / 100)
		dim = (width, height)
  
		# resize image
		img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
		cv2.imshow('result', img)
		# end

	#waitfor the user to press escape and break the while loop 
	k = cv2.waitKey(1) & 0xFF
	if k == 27:
		break
		
#destroys all window
cv2.destroyAllWindows()