#! /usr/bin/env python3

'''
@author: Emerson Martins de Andrade
COPPE/UFRJ     2025

Based on: https://sparklers-the-makers.github.io/blog/robotics/use-neo-6m-module-with-raspberry-pi/
'''

from threading import Thread

import serial
import pynmea2
import datetime
import pytz
from time import sleep

class GPS(Thread, object):

    def __init__(self):

        Thread.__init__(self)
        
        self.ser = None

        self.lat = 0
        self.lng = 0
        self.date_time = ""

        self._start_thread('main')
        
    def utc_to_local(self, utc_date, utc_time):
        tz_eastern = pytz.timezone('UTC')
        tz_brazil = pytz.timezone('Brazil/East')
        return tz_eastern.localize(datetime.datetime.strptime(str(utc_date)+str(utc_time).replace("+00:00",""), "%Y-%m-%d%H:%M:%S")).astimezone(tz_brazil).strftime("%Y-%m-%d,%H:%M:%S")


    def _start_thread(self, name):
        if name == 'main':
            self.main_loop = Thread(target=self._update)
            self.main_loop.daemon = True
            self.main_loop.start()

    def _update(self):
        while True:
            
            self.ser = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.5)
            
            newdata = self.ser.readline()
            
            try:
                newdata = str(newdata.decode("utf-8")).replace("b'","").replace("'","")
                if newdata[0:6] == "$GPRMC":
                    newmsg=pynmea2.parse(newdata)
                    self.lat = newmsg.latitude
                    self.lng = newmsg.longitude
                    self.date_time = self.utc_to_local(newmsg.datestamp, newmsg.timestamp)

            except Exception as e:
                print(e)
                pass
                
            sleep(2)
            self.ser.close()
            sleep(0.5)
    
