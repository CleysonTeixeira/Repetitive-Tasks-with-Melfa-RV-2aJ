#!/usr/bin/env python

import time
import RobotArm
import Communication
import StringHandler

robot = RobotArm.RobotArm()

robot.init() #inicia

robot.handOpen()

time.sleep(2) #delay

robot.handClose()

robot.turnOff() #desliga
