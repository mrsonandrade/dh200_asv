#! /usr/bin/env python3

'''
@author: Emerson Martins de Andrade
COPPE/UFRJ     2025

'''

from threading import Thread
from time import sleep
import serial

from sensors.Temperature_DS18B20 import DS18B20

class GetAllSensors(Thread, object):

    def __init__(self):

        Thread.__init__(self)

        self.ser = serial.Serial(port='/dev/ttyUSB0',
                                 baudrate = 9600,
                                 timeout=1)
        sleep(2)
        
        self.data = {}
        
        self.temperature_sensor = DS18B20()
        self.temperature_sensor._start_thread('main')

        self._start_thread('main')

    def _start_thread(self, name):
        if name == 'main':
            self.main_loop = Thread(target=self._update)
            self.main_loop.daemon = True
            self.main_loop.start()

    def _update(self):
        while 1:
            try:
                msg_received = self.ser.readline().strip()
                data = str(msg_received.decode("utf-8")).replace("b'","").replace("'","").split(",")
                
                for index_i in range(len(data)):
                    if data[index_i]=='ph':
                        self.data['ph'] = float(data[index_i+1])
                    if data[index_i]=='conductivity':
                        self.data['conductivity'] = float(data[index_i+1])
                
                self.data['temperature'] = self.temperature_sensor.temperature
                        
                sleep(1.2)
                            
            except Exception as e:
                print(e)
                pass
