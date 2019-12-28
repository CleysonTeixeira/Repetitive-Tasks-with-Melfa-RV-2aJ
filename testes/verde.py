#!/usr/bin/env python

import sys
import cv2	
import rospy	
import numpy	
import imutils 

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

import time
import RobotArm
import Communication
import StringHandler

bridge = CvBridge()
bgr_image = numpy.zeros((256, 256, 3), dtype = "uint8")

def callback(data):
    try:
        global bgr_image
        bgr_image = bridge.imgmsg_to_cv2(data, "bgr8")

    except CvBridgeError as e:
        print(e)

def retornaCalibracao(instanciaRospy, instanciaCv2):
    hmax = None
    hmin = None
    smax = None
    smin = None
    vmax = None
    vmin = None

    continuar = True

    while not instanciaRospy.is_shutdown() and continuar:
        hmax = instanciaCv2.getTrackbarPos('h_max', 'image')
        hmin = instanciaCv2.getTrackbarPos('h_min', 'image')
        smax = instanciaCv2.getTrackbarPos('s_max', 'image')
        smin = instanciaCv2.getTrackbarPos('s_min', 'image')
        vmax = instanciaCv2.getTrackbarPos('v_max', 'image')
        vmin = instanciaCv2.getTrackbarPos('v_min', 'image')

        hsv_image = instanciaCv2.cvtColor(bgr_image, instanciaCv2.COLOR_BGR2HSV)

        lower_hsv = numpy.array([hmin, smin, vmin])
        higher_hsv = numpy.array([hmax, smax, vmax])

        image = instanciaCv2.inRange(hsv_image, lower_hsv, higher_hsv)
        image = instanciaCv2.erode(image, None, iterations=2)
        image = instanciaCv2.dilate(image, None, iterations=2)

        mask = image.copy()
        frame = bgr_image.copy()

        instanciaCv2.imshow("image", image)
        instanciaCv2.imshow("frame", frame)

        key = instanciaCv2.waitKey(3)

        if key == ord("s"):
            continuar = False

    return [hmax, hmin, smax, smin, vmax, vmin]

def retornaCoordenadas(instanciaRospy, instanciaCv2, dadosCalibracao):
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    lower_hsv = numpy.array([dadosCalibracao[1], dadosCalibracao[3], dadosCalibracao[5]])
    higher_hsv = numpy.array([dadosCalibracao[0], dadosCalibracao[2], dadosCalibracao[4]])

    image = instanciaCv2.inRange(hsv_image, lower_hsv, higher_hsv)
    image = instanciaCv2.erode(image, None, iterations=2)
    image = instanciaCv2.dilate(image, None, iterations=2)

    mask = image.copy()
    frame = bgr_image.copy()

    contours = instanciaCv2.findContours(mask.copy(), instanciaCv2.RETR_EXTERNAL, instanciaCv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    center = None

    biggestContour = max(contours, key = instanciaCv2.contourArea)
    ((x, y), radius) = instanciaCv2.minEnclosingCircle(biggestContour)
    positionMoments = instanciaCv2.moments(biggestContour)
    center = (int(positionMoments["m10"] / positionMoments["m00"]), int(positionMoments["m01"] / positionMoments["m00"]))
    x = int(positionMoments["m10"] / positionMoments["m00"])
    y = int(positionMoments["m01"] / positionMoments["m00"])

    instanciaCv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 1)
    instanciaCv2.circle(frame, center, 3, (0, 0, 255), -1)

    return (x, y)

def aplicaTransformada(x, y):
    new_x = int((150 + y)/1.32926)
    new_y = int((x - 310)/1.27)
    if new_y <= -49:
    	new_y = new_y - 2
    if new_y <= -104:
    	new_x = new_x - 0.5
    if new_y <= -134:
    	new_y = new_y - 0.5
    	new_x = new_x - 1.2
    if new_y <= -219:
    	new_x = new_x - 0.2
    if new_x >= 357:
    	new_y = new_y - 1.2
    	if new_y <= -75:
    		new_y = new_y - 2.3
    #new_x = float(73 + y)
   # new_y = float(x - 305)
    return (new_x, new_y)


def moveRobo(robotInstance, x, y, z = 269.8):
    print 'Vou mudar'
    print x, y, z

    try:
        robotInstance.moveCartesianPosition(x, y, z, 180, 180, 50)
    except Exception, e:
        print 'DEU RUIM'
        print e

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

    calibracao = retornaCalibracao(rospy, cv2)
    print calibracao

    (x, y) = retornaCoordenadas(rospy, cv2, [103, 20, 255, 62, 213, 155]) 
    
    print x, y

    (a, b) = aplicaTransformada(x, y)

    print a, b

    cv2.destroyAllWindows()

    robot = RobotArm.RobotArm()
    
    #incia as tarefas do robo
    robot.init() #inicia a conexao
    moveRobo(robot, 150, 0, 400)

    robot.handClose() #abre a garra 

  
    moveRobo(robot, a, b) # localiza o disco e manda o robo

    robot.handOpen()

    print 'VOU EXECUTAR O PROXIMO MOVIMENTO'
    time.sleep(2)

    moveRobo(robot, 150, 0, 400)

    robot.handClose()

    robot.turnOff()

    print 'terminei'

if __name__ == '__main__':
    main(sys.argv)