#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

    '''
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
   '''

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
    if (new_x >= 344) and (new_y > -175):
        new_y = new_y - 2
        if new_y < -19:
            new_y = new_y -1
    if new_y <=-50:
        new_y = new_y - 2
    if (new_x >= 360) and (new_y >=22.5):
        new_y = new_y + 3
        if new_y >=75.5:
            new_y = new_y + 1
    if  (354<= new_x < 360) and new_y >= 46:
        new_y = new_y + 2.5
    if (313 <= new_x <=332) and (new_y < -3):
        new_y = new_y - 3
    if (new_x == 281) and (new_y == -4):
    	new_y = new_y + 1
    if (new_x >= 286 ) and (new_y <-44):
        new_y = new_y - 1
    if (new_x == 288) and (new_y == -3):
        new_y = -4
        if new_y == 77:
        	new_y = new_y + 1.5
    if (346 <= new_x <= 349) and (new_y == 63):
    	new_y = new_y + 2
    if (340 <= new_x <= 343) and (new_y == -34):
    	new_y = new_y - 3.5
    return (new_x, new_y)

def moveRobo(robotInstance, x, y, z = 269.8):
    #print 'Vou mudar'
    #print x, y, z

    try:
        robotInstance.moveCartesianPosition(x, y, z, 180, 180, 50)
    except Exception, e:
        #print 'DEU RUIM'
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
    
    cv2.destroyAllWindows()

    print 'Insira corretamente as cores das peças: branca, azul, verde e vermelha'
    
    primeira = raw_input("Insira a primeira peça: ")
    segunda = raw_input("Insira a segunda peça: ")
    terceira = raw_input("Insira a terceira peça: ")
    quarta = raw_input("Insira a quarta peça: ")

    branca = "branca"
    azul = "azul"
    vermelha = "vermelha"
    verde = "verde"

    #Determinando a primeira peça:

    if primeira == "branca":
        (x1, y1) = retornaCoordenadas(rospy, cv2, [88, 60, 255, 13, 255, 205] )
        (a1, b1) = aplicaTransformada(x1, y1)
    if primeira == "azul":
        (x1, y1) = retornaCoordenadas(rospy, cv2, [255, 88, 255, 89, 255, 184] )
        (a1, b1) = aplicaTransformada(x1, y1)
    if primeira == "verde":
        (x1, y1) = retornaCoordenadas(rospy, cv2, [94, 52, 238, 99, 222, 157] )
        (a1, b1) = aplicaTransformada(x1, y1)
    if primeira == "vermelha":
        (x1, y1) = retornaCoordenadas(rospy, cv2, [180, 0, 165, 107, 255, 219] )
        (a1, b1) = aplicaTransformada(x1, y1)   


    #Determinando a segunda peça:
    
    if segunda == "branca":
        (x2, y2) = retornaCoordenadas(rospy, cv2, [88, 60, 255, 13, 255, 205] )
        (a2, b2) = aplicaTransformada(x2, y2)
    if segunda == "azul":
        (x2, y2) = retornaCoordenadas(rospy, cv2, [255, 88, 255, 89, 255, 184] )
        (a2, b2) = aplicaTransformada(x2, y2)
    if segunda == "verde":
        (x2, y2) = retornaCoordenadas(rospy, cv2, [94, 52, 238, 99, 222, 157] )
        (a2, b2) = aplicaTransformada(x2, y2)
    if segunda == "vermelha":
        (x2, y2) = retornaCoordenadas(rospy, cv2, [180, 0, 165, 107, 255, 219] )
        (a2, b2) = aplicaTransformada(x2, y2)   


    #Determinando a terceira peça:
    
    if terceira == "branca":
        (x3, y3) = retornaCoordenadas(rospy, cv2, [88, 60, 255, 13, 255, 205] )
        (a3, b3) = aplicaTransformada(x3, y3)
    if terceira == "azul":
        (x3, y3) = retornaCoordenadas(rospy, cv2, [255, 88, 255, 89, 255, 184] )
        (a3, b3) = aplicaTransformada(x3, y3)
    if terceira == "verde":
        (x3, y3) = retornaCoordenadas(rospy, cv2, [94, 52, 238, 99, 222, 157] )
        (a3, b3) = aplicaTransformada(x3, y3)
    if terceira == "vermelha":
        (x3, y3) = retornaCoordenadas(rospy, cv2, [180, 0, 165, 107, 255, 219] )
        (a3, b3) = aplicaTransformada(x3, y3)   


    #Determinando a quarta peça:
    
    if quarta == "branca":
        (x4, y4) = retornaCoordenadas(rospy, cv2, [88, 60, 255, 13, 255, 205] )
        (a4, b4) = aplicaTransformada(x4, y4)
    if quarta == "azul":
        (x4, y4) = retornaCoordenadas(rospy, cv2, [255, 88, 255, 89, 255, 184] )
        (a4, b4) = aplicaTransformada(x4, y4)
    if quarta == "verde":
        (x4, y4) = retornaCoordenadas(rospy, cv2, [94, 52, 238, 99, 222, 157] )
        (a4, b4) = aplicaTransformada(x4, y4)
    if quarta == "vermelha":
        (x4, y4) = retornaCoordenadas(rospy, cv2, [180, 0, 165, 107, 255, 219] )
        (a4, b4) = aplicaTransformada(x4, y4)   
    

    robot = RobotArm.RobotArm()
    
    #incia as tarefas do robo
    robot.init() #inicia a conexao

    #Posição inicial
    moveRobo(robot, 150, 0, 400)

    robot.handClose() #abre a garra (ao contrário o nome)

    #altura das peças (Verificar) 
    z0 = 269.8
    z1 = z0 + 20
    z2 = z1 + 20
    z3 = z2 + 20

    #movendo o robô para a peça 1
    moveRobo(robot, a1, b1, z0)
    robot.handOpen()
    time.sleep(2)

    #posição onde será empilhada a peça 1
    moveRobo(robot, 150, 0, 400) 
    time.sleep(0.5)
    moveRobo(robot, 234, -120, z0)
    robot.handClose()
    moveRobo(robot, 150, 0, 400)

    print ' '
    print '-----------------------------'
    print 'Peça 1 empilhada com sucesso!'
    print '-----------------------------'
    time.sleep(0.5)

    #movendo o robô para a peça 2
    moveRobo(robot, a2, b2, z0)
    robot.handOpen()
    time.sleep(2)

    #posição onde será empilhada a peça 2
    moveRobo(robot, 150, 0, 400) 
    time.sleep(0.5)
    moveRobo(robot, 234, -120, z1)
    robot.handClose()
    moveRobo(robot, 150, 0, 400)
    
    print ' '
    print '-----------------------------'
    print 'Peça 2 empilhada com sucesso!'
    print '-----------------------------'
    time.sleep(0.5)

    #movendo o robô para a peça 3
    moveRobo(robot, a3, b3, z0)
    robot.handOpen()
    time.sleep(2)

    #posição onde será empilhada a peça 3
    moveRobo(robot, 150, 0, 400) 
    time.sleep(0.5)
    moveRobo(robot, 234, -120, z2)
    robot.handClose()
    moveRobo(robot, 150, 0, 400)
    
    print ' '
    print '-----------------------------'
    print 'Peça 3 empilhada com sucesso!'
    print '-----------------------------'

    time.sleep(0.5)
    
    #movendo o robô para a peça 4
    moveRobo(robot, a4, b4, z0)
    robot.handOpen()
    time.sleep(2)

    #posição onde será empilhada a peça 4
    moveRobo(robot, 150, 0, 400) 
    time.sleep(0.5)
    moveRobo(robot, 234, -120, z3)
    robot.handClose()
    moveRobo(robot, 150, 0, 400)
    
    print ' '
    print '-----------------------------'
    print 'Peça 4 empilhada com sucesso!'
    print '-----------------------------'
    time.sleep(0.5)

    robot.turnOff()

    print ' '
    print '-----------------------------'
    print '     Processo Finalizado!'
    print '     Peças Empilhadas'
    print '-----------------------------'

if __name__ == '__main__':
    main(sys.argv)
