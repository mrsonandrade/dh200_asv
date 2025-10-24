#! /usr/bin/env python3

'''
@author: Emerson Martins de Andrade
COPPE/UFRJ     2025
'''

import threading
import pigpio
import time
from time import sleep

class DifferentialPropelledRobot(object):

    def __init__(self):
      
        self.MOTOR_LEFT = 12
        self.MOTOR_RIGHT = 13
      
        self.pi = pigpio.pi()
        
        self.isbusy = False
        self._lock = threading.Lock()
        self._thread = None

    def calibrate(self):
        # calibrate ESC
        self.pi.set_servo_pulsewidth(self.MOTOR_LEFT, 2000)
        self.pi.set_servo_pulsewidth(self.MOTOR_RIGHT, 2000)
        time.sleep(2)
        self.pi.set_servo_pulsewidth(self.MOTOR_LEFT, 1000)
        self.pi.set_servo_pulsewidth(self.MOTOR_RIGHT, 1000)
        time.sleep(2)
        
    def move(self, leftDutyCycle, rightDutyCycle, delay=0):
        with self._lock:
            if self.isbusy:
                print('propulsion busy')
                return None
            self.isbusy = True
        
        self._thread = threading.Thread(target=self.move_in_thread, args=(leftDutyCycle, rightDutyCycle, delay,))
        self._thread.start()
        return self._thread

    def move_in_thread(self, leftDutyCycle, rightDutyCycle, delay=0):
        
        if leftDutyCycle<=0:
            left_pw = 1000
        elif leftDutyCycle>0:
            left_pw = int(1000 + 10*leftDutyCycle)
            
        if rightDutyCycle<=0:
            right_pw = 1000
        elif rightDutyCycle>0:
            right_pw = int(1000 + 10*rightDutyCycle)

        self.pi.set_servo_pulsewidth(self.MOTOR_LEFT, left_pw)
        self.pi.set_servo_pulsewidth(self.MOTOR_RIGHT, right_pw)
        
        sleep(delay)
        if delay==0:
          pass
        else:
          self.stop()
            
        with self._lock:
            self.isbusy = False
    
    def stop(self):
        self.pi.set_servo_pulsewidth(self.MOTOR_LEFT, 1000)
        self.pi.set_servo_pulsewidth(self.MOTOR_RIGHT, 1000)


