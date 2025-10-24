#! /usr/bin/env python3

'''
@author: Emerson Martins de Andrade
COPPE/UFRJ     2025
Tested in Python 3.11.2
'''

# Did you start the pigpio daemon? E.g.
# sudo pigpiod

from DifferentialPropelledRobot import DifferentialPropelledRobot
from Robot import Robot # high-level robot logic
from XBee import XBee
from GPS import GPS
from Compass import Compass
from Sensors import GetAllSensors
from time import sleep

####################################################################
RobotID = '15' # CHANGE THIS ID FOR EACH ROBOT
####################################################################

Position = GPS()
Orientation = Compass()
Communication = XBee()
Motion = DifferentialPropelledRobot()
Sensors = GetAllSensors()

robot_kw = {'RobotID': RobotID,
            'Communication': Communication,
            'Motion': Motion,
            'Position': Position,
            'Orientation': Orientation,
            'Sensors': Sensors}

robot = Robot(**robot_kw) # high-level robot logic

threads = [Position, Orientation, Communication, Sensors, robot]
labels = ['Position','Orientation','Communication', 'Sensors', 'Robot']

print('Robot initialization complete!')

while True:

    for thread, label in zip(threads, labels):
        if not thread.main_loop.is_alive():
            msg = 'Restarted Main Thread in %s' % label
            thread._start_thread('main')
            print(msg)

    sleep(10)
