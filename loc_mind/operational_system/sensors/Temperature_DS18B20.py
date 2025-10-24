#! /usr/bin/env python3

'''
@author: Emerson Martins de Andrade
COPPE/UFRJ     2025
'''

from threading import Thread
from time import sleep
from w1thermsensor import W1ThermSensor

class DS18B20(Thread, object):

    def __init__(self):

        Thread.__init__(self)

        self.sensor = W1ThermSensor()
        self.temperature = 0.0

        self._start_thread('main')

    def _start_thread(self, name):
        if name == 'main':
            self.main_loop = Thread(target=self._update)
            self.main_loop.daemon = True
            self.main_loop.start()

    def _update(self):
        while 1:
            try:
                self.temperature = self.sensor.get_temperature()
            except Exception as e:
                print(e)
                pass
            sleep(1)
