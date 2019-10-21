# MasterThesis

Methodology:
  - Follow instructions from gctronic webpage (https://www.gctronic.com/doc/index.php/E-Puck) and download python packages
  - Download python 2.7
  - Download Microsoft Visual Studio 2010 (in order to have Microsoft SDK version v7.0A) and Microsoft Visual Studio C++ 2008
  - Place an overhead camera (I used Windows Webcam) and Aruco Tags (https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html, http://chev.me/arucogen/) on top of the e-puck robots
  - Select position 3 on e-puck robots and connect to the computer using the e-puck numbers as password
  - Find MAC addresses of the e-puck robots in device manager

Python codes:
  - Calibrate.py and CHESSBOARD.png calibrate the camera
  - CaptureTarget2circ.py and CaptureTarget3circ.py implement a dynamic balanced circular formation with 2 or 3 e-puck robots moving and one e-puck robot as the center of the formation
  - Rand3.py implements a random motion with 3 e-puck robots

Matlab code:
Simulates the experiments with matlab. Nc and Nr can be changed in order to modify the number of robots performing the circular formation (Nc) and the number of robots moving randomly (Nr).
