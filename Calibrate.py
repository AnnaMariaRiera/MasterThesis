# Calibration of the camera. We need a chessboard printed in a paper and we need to place it at different locations and orientations. The program will have to fins the chessboard 10 times. This creates a file where the calibration values will be stored. Then, the values from this file will be used in the actual programs that perform the motion that we are interested in.

import numpy as np
import cv2    # sembla un paquet per tractar imatges, videos etc
import yaml


# termination criteria (eps = how close, max iter = maximum number of iterations, o una cosa aixi)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) 

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)  # Crea matriu de zeros de dimensio 42x3
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)  # Fa que aquesta matriu sigui [[0 0 0]; [1 0 0]; ... ; [6 0 0]; [0 1 0]; [1 1 0]; ... [6 1 0]; ... ; [6 5 0]]

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

cap = cv2.VideoCapture(0) # We creat a VideoCapture object. This will return video from the first webcam (0) of our computer
found = 0
while(found < 10):  # Here, 10 can be changed to whatever number you like to choose
    ret, img = cap.read() # Capture frame-by-frame. ret is a boolean that indicates whether frame is read correctly. img is the image returned. If there is no 
						  # image it won't return an error, we will get None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Here we define the new variable gray as the frame (image) converted to grayscale. Note that openCV reads 
												  # colours as BGR (blue green red) instead of RGB
	
	
    
	# Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
		objpoints.append(objp)  # Certainly, every loop objp is the same, in 3D.

		corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
		imgpoints.append(corners2)

        # Draw and display the corners
		img = cv2.drawChessboardCorners(img, (7,6), corners2, ret)
		found += 1
		print found

    cv2.imshow('img', img) # Despite being a video stream, we still have to use imshow() to show it. We are displaying the img (not greyscale)
    cv2.waitKey(10)  # Playing video from file: cv2.waitKey(10) changes the time at which the video is shown. 10 is fast, but okay.


# When everything done, release the capture
cap.release()  # Releases the webcam
cv2.destroyAllWindows()  # closes all the imshow() windows

# So now we have our object points and image points and we are ready to go for calibration. For that we use the function cv2.calibrateCamera(). It returns the camera matrix, distortion coefficients, rotation and translation vectors etc.
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)



# # It's very important to transform the matrix to list.
# data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}
#
# with open("calibration.yaml", "w") as f:
#     yaml.dump(data, f)


cv_file = cv2.FileStorage("calib.yaml", cv2.FILE_STORAGE_WRITE)
cv_file.write("camera_matrix", mtx)
cv_file.write("dist_coeff", dist)
# note you *release* you don't close() a FileStorage object
cv_file.release()
