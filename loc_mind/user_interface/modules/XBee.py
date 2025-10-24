#! /usr/bin/env python3

'''
@author: Emerson Martins de Andrade
COPPE/UFRJ     2025
@version: 1.0.0

Based on: https://circuitdigest.com/microcontroller-projects/raspberry-pi-xbee-module-interfacing
'''

from threading import Thread
import serial

from .utils_communication import process_data

class NoSerial():

    def __init__(self):

        self.a = 0

    def isOpen(self):
        return True

    def readline(self):
        return str.encode('')

        

class XBee(Thread, object):

    def __init__(self):

        Thread.__init__(self)

        try:
            self.ser = serial.Serial(port='COM3',
                                 baudrate = 9600,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS,
                                 timeout=0.1)
        except:
            print('no serial port detected.')
            self.ser = NoSerial()
        
        self.data = None

        self._start_thread('main')

    def _start_thread(self, name):
        if name == 'main':
            self.main_loop = Thread(target=self._update)
            self.main_loop.daemon = True
            self.main_loop.start()

    def send_message(self, msg):
        self.ser.write(str.encode(msg))

    def _update(self):
        while 1:
            try:
                msg_received = self.ser.readline().strip()
                data = str(msg_received.decode("utf-8")).replace("b'","").replace("'","").split(",")

                #### begin fake msg - COMMENT if needed
                #msg_received = "robot_id,%s,pose,%.2f,%.2f,%.2f,%.6f,lat,%.6f,lng,%.6f,datetimegps,%s" % ("10",300.0,150.0,48.0,124465435.46,-22.863605987551466,-43.214345755956224,"time:data:")
                #data = msg_received.split(",")
                #sleep(1)
                #print(data)
                #### end fake msg

                data = process_data(data)
                if self.data==None:
                    self.data = data
                else:
                    for id_i in list(data.keys()):
                        if id_i not in list(self.data.keys()):
                            self.data[id_i] = {}
                        for key_i in list(data[id_i].keys()):
                            self.data[id_i][key_i] = data[id_i][key_i]

            except Exception as e:
                print(e)
