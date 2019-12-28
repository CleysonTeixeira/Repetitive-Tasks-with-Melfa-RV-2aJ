#!/usr/bin/env python

import sys
import cv2	
import rospy	
import numpy	
import imutils 

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

bridge = CvBridge()
bgr_image = numpy.zeros((256, 256, 3), dtype = "uint8")

points = deque([], maxlen = 50000)
drawPath = False

def callback(data):
    try:
        global bgr_image
        bgr_image = bridge.imgmsg_to_cv2(data, "bgr8")

    except CvBridgeError as e:
        print(e)

def main(args):

    image_sub = rospy.Subscriber("camera/rgb/image_color", Image, callback) 

    rospy.init_node('image_converter', anonymous = True)

    cv2.namedWindow('image')
    cv2.createTrackbar('h_max','image', 0, 255, lambda args: None)
    cv2.createTrackbar('h_min','image', 0, 255, lambda args: None)
    cv2.createTrackbar('s_max','image', 0, 255, lambda args: None)
    cv2.createTrackbar('s_min','image', 0, 255, lambda args: None)
    cv2.createTrackbar('v_max','image', 0, 255, lambda args: None)
    cv2.createTrackbar('v_min','image', 0, 255, lambda args: None)

    while not rospy.is_shutdown() :

        hmax = cv2.getTrackbarPos('h_max', 'image')
        hmin = cv2.getTrackbarPos('h_min', 'image')
        smax = cv2.getTrackbarPos('s_max', 'image')
        smin = cv2.getTrackbarPos('s_min', 'image')
        vmax = cv2.getTrackbarPos('v_max', 'image')
        vmin = cv2.getTrackbarPos('v_min', 'image')

        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        lower_hsv = numpy.array([hmin, smin, vmin])
        higher_hsv = numpy.array([hmax, smax, vmax])

        image = cv2.inRange(hsv_image, lower_hsv, higher_hsv)
        image = cv2.erode(image, None, iterations=2)
        image = cv2.dilate(image, None, iterations=2)

        mask = image.copy()
        frame = bgr_image.copy()

        global drawPath

        #if drawPath :
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        center = None

        if len(contours) > 0:
            biggestContour = max(contours, key = cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(biggestContour)
            positionMoments = cv2.moments(biggestContour)
            center = (int(positionMoments["m10"] / positionMoments["m00"]), int(positionMoments["m01"] / positionMoments["m00"]))

            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 1)
            v2.circle(frame, center, 3, (0, 0, 255), -1)
	
        
	print center

	arq = open('/tmp/lista.txt', 'w')
	texto = []
	texto.append(center "\n")
	arq.writelines(texto)
	arq.close()

        if not center == None :
            points.appendleft(center)

        for i in range(1, len(points)):
            if points[i - 1] is not None or points[i] is not None:
                 cv2.line(frame, points[i - 1], points[i], (0, 0, 255), 2)

        cv2.imshow("image", image)
        cv2.imshow("frame", frame)
        key = cv2.waitKey(3)

        if key == ord("c"):
            points.clear()

       # if key == ord("s"):
            drawPath = not drawPath

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
