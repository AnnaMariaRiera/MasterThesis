# Circular motion using UMC2 with 2 vehicles (third vehicle is the center of the formation)

from ePuck import ePuck
import sys
import numpy as np
import cv2
import cv2.aruco as aruco
import math
import matplotlib as mat
import matplotlib.pyplot as plt
import time

#specify v and w in m/s and rad/s

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).

def satv(v): # Sets the velocity to the minimum one (closest to 0 pos or neg) 

    vmax=0.13
    vmin=-0.13

    for i in range(0,len(v)):
        v[i] = min(vmax, max(vmin, v[i]))

    return v
def satw(w): # Sets the angular velocity to the minimum one (closest to 0 pos or neg) 

    wmax = 4.5
    wmin = -4.5

    for i in range(0, len(w)):
        w[i] = min(wmax, max(wmin, w[i]))

    return w

# CALCULATE ROTATION ANGLES
def rotationMatrixToEulerAngles(R):
	# transforms the rotation matrix R into an array of [x, y, z] the Euler angles
	
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

# COMMUNICATE VELOCITY COMMANDS
def setvel(v, w):
	# Calculates the velocity of every wheel (left and right) for the 3 robots (robots 1, 2 and 3)
	# and sets them (les posa als robots)
	
    # assert v < 0.12

    L = 0.053  # Axel Width
    R = 0.0205  # Wheel Radius

    vr1 = (2 * v[0] + w[0] * L) / (2 * R)
    vl1 = (2 * v[0] - w[0] * L) / (2 * R)
    rs1 = vr1 / 0.00628
    ls1 = vl1 / 0.00628

    vr2 = (2 * v[1] + w[1] * L) / (2 * R)
    vl2 = (2 * v[1] - w[1] * L) / (2 * R)
    rs2 = vr2 / 0.00628
    ls2 = vl2 / 0.00628

    vr3 = (2 * v[2] + w[2] * L) / (2 * R)
    vl3 = (2 * v[2] - w[2] * L) / (2 * R)
    rs3 = vr3 / 0.00628
    ls3 = vl3 / 0.00628

    print(ls1)
    print(rs1)

    robot1.set_motors_speed(ls1, rs1)
    robot1.step()
    robot2.set_motors_speed(ls2, rs2)
    robot2.step()
    robot3.set_motors_speed(ls3, rs3)
    robot3.step()

# LIST OF E-PUCKS : CHANGE MAC ADDRESSES : DEVICE MANAGER
epucks = {
#    '3303': '10:00:e8:c5:64:37',
#    '3214': '10:00:e8:c5:64:56',
#    '3281': '10:00:e8:c5:61:82',
#    '3276': '10:00:e8:c5:61:43',
#    '3109': '10:00:e8:ad:78:1d'
	'3111': '10:00:E8:C5:61:B2',
	'3276': '10:00:E8:C5:61:43',
	'3279': '10:00:E8:AD:78:24',
	'3281': '10:00:E8:C5:61:82',
	'3282': '10:00:E8:C5:64:3A',
	'3302': '10:00:E8:AD:78:22',
	'3305': '10:00:E8:C5:61:A7'
}

def log(text):
    """	Show @text in standart output with colors """

    blue = ' '  # '\033[1;34m'
    off = '  '  # '\033[1;m'

    print(''.join((blue, '[Log] ', off, str(text))))
def error(text):
	red = ' '  # '\033[1;31m'
	off = '  '  # '\033[1;m'
	
	print(''.join((red, '[Error] ', off, str(text))))


# 1. START AND SET UP VIDEO
cap = cv2.VideoCapture(0)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')   # To record and save the recording to a new file
out = cv2.VideoWriter('output.avi',fourcc, 30.0, (640,480))

cv_file = cv2.FileStorage("calib.yaml", cv2.FILE_STORAGE_READ) # read calibration file
		# note we also have to specify the type to retrieve otherwise we only get a FileNode object back instead of a matrix
mtx = cv_file.getNode("camera_matrix").mat()
dist = cv_file.getNode("dist_coeff").mat()

markerLength = 0.08   # SET MARKER LENGTH IN (M)

print('Connecting with the ePuck')

# CONNECT WITH E-PUCKS - ALL LEDS LIGHT WHEN CONNECTED
try:
	# First, create an ePuck object.
    # If you want debug information:
    # ~ robot = ePuck(mac, debug = True)
    # else:
    # robot1 = ePuck(epucks['3109'])
    # robot2 = ePuck(epucks['3276'])
    # robot3 = ePuck(epucks['3214'])
	
	robot1 = ePuck(epucks['3279'])
	robot2 = ePuck(epucks['3302'])
	robot3 = ePuck(epucks['3305'])
	# Second, connect to it
	robot1.connect()
	robot2.connect()
	robot3.connect()
	
	# You can enable various sensors at the same time. Take a look to DIC_SENSORS for know the name of the sensors
	for i in range(1, 8):  # turn on all lights
		robot1.set_led(i, 1)
		robot2.set_led(i, 1)
		robot3.set_led(i, 1)
	
	log('Conection complete. CTRL+C to stop')
	log('Library version: ' + robot1.version)
except Exception, e:
    error(e)
    sys.exit(1)

roboid1 = 1 
roboid2 = 2
roboid3 = 3
epuckids = [roboid1, roboid2, roboid3]
n = 3
p = 1000   # number of iterations that we want to conduct

stoptag = 27

tvecs = np.zeros((n, 3))  # matrix nx3
rvecs = np.zeros((n, 3))
angles = np.zeros((n, 3))
w = np.zeros((n, 1))
v = np.zeros((n, 1))
x = np.zeros((p, n))
y = np.zeros((p, n))
t = np.zeros((p, 1))
vrecord = np.zeros((p, n))
wrecord = np.zeros((p, n))
told=time.time()   # time.time() returns the time in seconds since the epoch as a floating point number.  The epoch is the point where the time starts. On January 1st of that year, at 0 hours, the "time since the epoch" is zero. For Unix, the epoch is 1970. To find out what the epoch is, look at gmtime(0).

count=0
distance = 0.3  # radius of the formation
kv = 0.5
wv = 1   # kw
vmax = 0.13
wmax = 1

phi = -math.atan(distance*wv*wmax/(kv*vmax))

theta = np.zeros((n, 1))

while (True):

	ret, frame = cap.read()  # read video from the camera
    # operations on the frame come here
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # frame transformed to greyscale
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
	parameters = aruco.DetectorParameters_create()

    # lists of ids and the corners beloning to each id
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dist)  # For a single marker

	# INITIAL STEP. Important: when you execute 'step()', al sensors and actuators are updated. All changes you do on the ePuck will be 
	# effectives after this method, not before
	robot1.step()
	robot2.step()
	robot3.step()
	
	if (count<p):
		if np.all(ids != None):  # if all ids are different of "None"
			if (all(i in ids for i in epuckids)):  # si tots els ids dels epucks que tenc en marxa els esta detectant la camera
				for i in range(0, n):
					tvecs[i] = (tvec[np.where(ids == epuckids[i])][0])  # TVECS=POSITION  -> tvecs[n-1][1, 2 o 3] position of the target
					rvecs[i] = (rvec[np.where(ids == epuckids[i])][0])
					dst, jacobian = cv2.Rodrigues(rvecs[i])
					angles[i] = rotationMatrixToEulerAngles(dst) # ANGLES=ROTATION ANGLES
					
					g=1
					for j in range(0,n-1):
						if j!=i:
							a = ((math.atan2(tvecs[j][1] - tvecs[i][1], tvecs[j][0] - tvecs[i][0]) + 2 * math.pi) % (2 * math.pi) - (angles[i][2] + 2 * math.pi) % (2 * math.pi) +2*math.pi)%(2*math.pi)  # beta_ij
							g=g-math.cos(a)
							
					b = ((math.atan2(tvecs[n - 1][1] - tvecs[i][1], tvecs[n - 1][0] - tvecs[i][0]) + 2 * math.pi) % (2 * math.pi) - (angles[i][2] + 2 * math.pi) % (2 * math.pi) +2*math.pi)%(2*math.pi)  # alpha_i
						
					aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)  # Draw Axis

					v[i] = g*kv * vmax * math.cos(b + phi)
					w[i] = -g*wv * wmax * math.sin(b + phi)
					
					x[count][i]=tvecs[i][0]
					y[count][i]=tvecs[i][1]

				v[2]=0
				w[2]=0
				v=satv(v)
				w=satw(w)
				vrecord[count] = v.transpose()
				wrecord[count] = w.transpose()
				setvel(v, w)
				t[count] = time.time() - told

				count+=1

			if stoptag in ids:
				setvel([0, 0, 0], [0, 0, 0])
				plt.plot(x,y)
				plt.show()
				exit(6)
	else:
		setvel([0, 0, 0], [0, 0, 0])
		np.savetxt('nc2/x_nc2.csv', x, delimiter=",")
		np.savetxt('nc2/y_nc2.csv', y, delimiter=",")
		np.savetxt('nc2/time_nc2.csv', t, delimiter=",")
		np.savetxt('nc2/v_nc2.csv', vrecord, delimiter=",")
		np.savetxt('nc2/w_nc2.csv', wrecord, delimiter=",")
		plt.plot(x, y)
		plt.show()
		exit(6)

	frame = aruco.drawDetectedMarkers(frame, corners)
	font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)
	
    ###### DRAW ID #####
	cv2.putText(frame, "Id: " + str(ids), (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
	
	cv2.imshow('frame', frame)
	out.write(frame)
	
	if cv2.waitKey(1) & 0xFF == ord('q'):   # This statement just runs once per frame. Basically, if we get a key, and that key is a q, we will exit the
											# while loop with a break
		break

# When everything done, release the capture
cap.release()    # release figure
out.release()    # release video
cv2.destroyAllWindows()
