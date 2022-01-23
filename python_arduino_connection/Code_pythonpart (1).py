from picamera.array import PiRGBArray
from picamera import PiCamera
from collections import deque
import argparse
import time
import cv2
import sys
import imutils
import serial

control= serial.Serial('/dev/ttyACM0',115200)
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution=(640,480)
camera.framerate = 4
rawCapture = PiRGBArray(camera, size=(640, 480))

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=4, help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points 
greenLower = (128, 128, 128)
greenUpper = (255, 255, 255)
pts = deque(maxlen=args["buffer"])

i = 0
j = 0
k = 0
m = 0
n = 0


# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr"):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array 


    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(image, width=600)
    # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
# find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    #print(cnts)
# only proceed if at least one contour was found
    if len(cnts) > 0:
	    # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            x1=center[0]
            y1=center[1]
            
            if x1 > 400 and y1 >= 200:
                i+=1
                print i
                if i < 5:
                    print('R wait')
                else:
                    control.write(b'4')# Right
                    print('right')
                    i=0
            elif x1 < 200 and y1 >=200:
                j+=1
                print j
                if j < 5:
                    print('L wait')
                else:
                    control.write(b'3') # Left
                    print('left')
                    j=0
            elif  460 >= x >= 240 and y1 >= 200:
                k+=1
                print k
                if k<3:
                    print('Str wait')
                else:
                    control.write(b'1') # Forward
                    print('straight')
                    k=0
            elif  460 >= x >= 200 and y1 < 90:
                m+=1
                print m
                if m<3:
                    print('GF wait')
                else:
                    control.write(b'5')
                    print('gripper forward')
                    m=0
            elif  460 >= x >= 200 and 160 >= y1 >= 120:
                n+=1
                print n
                if n<3:
                    print('GB wait')
                else:
                    control.write(b'6')
                    print('gripper back')
                    n=0
            else:
                control.write(b'2') # Pause
                print('GF wait')
                print('gripper forward')
                
            print('X coordinate:',x1, 'Y coordinate',y1)
            # only proceed if the radius meets a minimum size
            if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
 
    # update the points queue
    pts.appendleft(center)

 
    # show the frame to our screen
    cv2.imshow("Frame", frame)

    key = cv2.waitKey(1)
    rawCapture.truncate(0)

    if key & 0xFF == ord('q'):
        break
    

cv2.destroyWindow("img")
cv2.destroyAllWindows() 
