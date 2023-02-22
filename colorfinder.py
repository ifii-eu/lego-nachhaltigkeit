#import opencv and numpy
import cv2  
import numpy as np

cap = cv2.VideoCapture(1,cv2.CAP_DSHOW) #video aufzeichhnung

#trackbar callback fucntion to update HSV value
def callback(x):
	global H_low,H_high,S_low,S_high,V_low,V_high
	#assign trackbar position value to H,S,V High and low variable
	H_low = cv2.getTrackbarPos('low H','controls')
	H_high = cv2.getTrackbarPos('high H','controls')
	S_low = cv2.getTrackbarPos('low S','controls')
	S_high = cv2.getTrackbarPos('high S','controls')
	V_low = cv2.getTrackbarPos('low V','controls')
	V_high = cv2.getTrackbarPos('high V','controls')

	minThreshold = cv2.getTrackbarPos('minThreshold','controls')
	maxThreshold = cv2.getTrackbarPos('maxThreshold','controls')
	boolean: filterByArea = cv2.getTrackbarPos('filterByArea','controls')
	minArea = cv2.getTrackbarPos('minArea','controls')
	boolean: filterByCircularity = cv2.getTrackbarPos('filterByCircularity','controls')
	#minCircularity = cv2.getTrackbarPos('minCircularity','controls')
	boolean: filterByConvexity = cv2.getTrackbarPos('filterByConvexity','controls')
	#minConvexity = cv2.getTrackbarPos('minConvexity','controls')
	boolean: filterByInertia = cv2.getTrackbarPos('filterByInertia','controls')
	#minInertiaRatio = cv2.getTrackbarPos('minInertiaRatio','controls')

#create a seperate window named 'controls' for trackbar
cv2.namedWindow('controls',2)
cv2.resizeWindow("controls", 550,10);

#global variable
H_low = 0 #Default: 0 #Besser: 21
H_high = 179 #Default: 179 #Besser: 30   
S_low= 0 #Default: 0 #Besser: 47
S_high = 255 #Default: 255 #Besser: 255
V_low= 0 #Default: 0 #Besser: 122 
V_high = 255 #Default: 255 #Besser: 255

#create trackbars for parameters
cv2.createTrackbar('minThreshold','controls',0,20,callback)
cv2.createTrackbar('maxThreshold','controls',0,400,callback)
cv2.createTrackbar('filterByArea','controls',False,True,callback)
cv2.createTrackbar('minArea','controls',0,3000,callback)
cv2.createTrackbar('filterByCircularity','controls',False,True,callback)
#cv2.createTrackbar('minCircularity','controls', 0.005 , 0.2 , callback)
cv2.createTrackbar('filterByConvexity','controls',False,True,callback)
#cv2.createTrackbar('minConvexity','controls', 0.1 , 5 ,callback)
cv2.createTrackbar('filterByInertia','controls',False,True,callback)
#cv2.createTrackbar('minInertiaRatio','controls', 0.0005 , 0.02 ,callback)

#create trackbars for high,low H,S,V 
cv2.createTrackbar('low H','controls',0,179,callback)
cv2.createTrackbar('high H','controls',179,179,callback)

cv2.createTrackbar('low S','controls',0,255,callback)
cv2.createTrackbar('high S','controls',255,255,callback)

cv2.createTrackbar('low V','controls',0,255,callback)
cv2.createTrackbar('high V','controls',255,255,callback)

while(1):
	#read source image
	img=cv2.imread("lego.jpg")
	ret, img = cap.read()
	#convert sourece image to HSC color mode
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	#hsv
	hsv_low = np.array([H_low, S_low, V_low], np.uint8)
	hsv_high = np.array([H_high, S_high, V_high], np.uint8)

	#making mask for hsv range
	mask = cv2.inRange(hsv, hsv_low, hsv_high)
	#print (mask)
        
	# Führe Rauschentfernung auf der Maske durch
	kernel = np.ones((5,5), np.uint8)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

	#masking HSV value selected color becomes black
	res = cv2.bitwise_and(img, img, mask=mask)      

	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()
        

	#Achtung! Vorher entfernen, damit die Regler funktionieren!
	H_low = 21 #Default: 0 #Besser: 21
	H_high = 30 #Default: 179 #Besser: 30   
	S_low= 47 #Default: 0 #Besser: 47
	S_high = 255 #Default: 255 #Besser: 255
	V_low= 122 #Default: 0 #Besser: 122 
	V_high = 255 #Default: 255 #Besser: 255


	# Change thresholds
	minThreshold = 10  #Default: 10  #ändert nichts?
	maxThreshold = 200  #Default: 200  #ändert nichts?

	# Filter by Area - Nach Fläche filtern
	# This is to avoid any identification of any small dots present in the image that can be wrongly detected as a circle. 
	# Damit sollen kleine Punkte im Bild, die fälschlicherweise als Kreis erkannt werden könnten, nicht erkannt werden.
	filterByArea = True #Default: True  #False ändert sehr viel!!!
	minArea = 50  #Default: 1500 #Besser: 50 damit auch noch bei viel Abstand die kleinen Steine erkannt werden! 

	# Filter by Circularity
	# This helps to identify, shapes that are more similar to a circle. 
	# Dies hilft, Formen zu erkennen, die einem Kreis ähnlicher sind.
	filterByCircularity = True  #Default: True  #False ändert nichts
	minCircularity = 0.0001  #Default: 0.1  #Besser: 0.0001

	# Filter by Convexity
	# Concavity in general, destroys the circularity. More is the convexity, the closer it is to a close circle. 
	# Konkavität zerstört im Allgemeinen die Kreisform. Je konvexer die Form ist, desto näher ist sie an einem geschlossenen Kreis
	filterByConvexity = True  #Default: True  #False ändert nichts
	minConvexity = 0.0001  #Default: 0.87  #Besser: 0.0001

	# Filter by Inertia
	# Objects similar to a circle has larger inertial.E.g. for a circle, this value is 1, for an ellipse it is between 0 and 1, and for a line it is 0. To filter by inertia ratio, set filterByInertia = 1, and set, 0 <= minInertiaRatio <= 1 and maxInertiaRatio (<=1 ) appropriately. 
	# Objekte, die einem Kreis ähneln, haben ein größeres Trägheitsverhältnis, z. B. für einen Kreis ist dieser Wert 1, für eine Ellipse liegt er zwischen 0 und 1 und für eine Linie ist er 0. Um nach dem Trägheitsverhältnis zu filtern, setzen Sie filterByInertia = 1 und setzen Sie 0 <= minInertiaRatio <= 1 und maxInertiaRatio (<=1 ) entsprechend.
	filterByInertia = True  #Default: True  #False ändert nichts
	minInertiaRatio = 0.0001  #Default: 0.01  #Besser: 0.0001

	# Change thresholds
	params.minThreshold = minThreshold
	params.maxThreshold = maxThreshold

	# Filter by Area.
	params.filterByArea = filterByArea
	params.minArea = minArea

	# Filter by Circularity
	params.filterByCircularity = filterByCircularity
	params.minCircularity = minCircularity

	# Filter by Convexity
	params.filterByConvexity = filterByConvexity
	params.minConvexity = minConvexity

	# Filter by Inertia
	params.filterByInertia = filterByInertia
	params.minInertiaRatio = minInertiaRatio

	# Create a detector with the parameters
	# OLD_not_working: detector = cv2.SimpleBlobDetector()
	# OLD_working:	detector = cv2.SimpleBlobDetector_create()
	detector = cv2.SimpleBlobDetector_create(params)

	mask=cv2.bitwise_not(mask)

	# Detect blobs.
	keypoints = detector.detect(mask)

	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
	im_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	# Show keypoints
	cv2.imshow("Keypoints", im_with_keypoints)

	#show other images
	cv2.imshow('mask',mask)
	cv2.imshow('res',res)

	

		
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

		# Find the keypoints that belong to the yellow rectangle
		yellow_keypoints = []
		for kp in keypoints:
			if (x_min <= kp.pt[0] <= x_max) and (y_min <= kp.pt[1] <= y_max):
				yellow_keypoints.append(kp)

		# Calculate the center of the yellow rectangle
		x_center = int((x_min + x_max) / 2)
		y_center = int((y_min + y_max) / 2)

		# Calculate the angle between each yellow brick and the center of the yellow rectangle
		angle_dict = {}
		for kp in yellow_keypoints:
			x, y = kp.pt
			angle = np.arctan2(y - y_center, x - x_center)
			angle_dict[kp] = angle

		# Sort the yellow bricks by their angle
		sorted_bricks = sorted(yellow_keypoints, key=lambda kp: angle_dict[kp])

		# Assign each brick to a quadrant based on its angle
		left_legos = []
		right_legos = []
		top_legos = []
		bottom_legos = []
		for kp in sorted_bricks:
			angle = angle_dict[kp]
			if np.pi / 4 <= angle < 3 * np.pi / 4:
				bottom_legos.append((int(kp.pt[0]), int(kp.pt[1])))
			elif -3 * np.pi / 4 <= angle < -np.pi / 4:
				top_legos.append((int(kp.pt[0]), int(kp.pt[1])))
			elif np.pi / 4 > angle >= -np.pi / 4:
				left_legos.append((int(kp.pt[0]), int(kp.pt[1])))
			else:
				right_legos.append((int(kp.pt[0]), int(kp.pt[1])))


		# Draw circles at the positions of each Lego brick in the picture
		for kp in keypoints:
			cv2.circle(img, (int(kp.pt[0]), int(kp.pt[1])), 10, (255, 255, 255), -1)

		# Draw circles at the positions of each Lego brick in the picture separated by side
		for position in left_legos:
			cv2.circle(img, position, 10, (0, 255, 0), -1)  # left - green

		for position in right_legos:
			cv2.circle(img, position, 10, (255, 0, 0), -1)  # right - blue

		for position in top_legos:
			cv2.circle(img, position, 10, (255, 255, 0), -1)  # top - cyan

		for position in bottom_legos:
			cv2.circle(img, position, 10, (0, 255, 255), -1)  # bottom - yellow





		# show result
		cv2.imshow('result', img)
		
		


	#waitfor the user to press escape and break the while loop 
	k = cv2.waitKey(1) & 0xFF
	if k == 27:
		break
		
#destroys all window
cv2.destroyAllWindows()