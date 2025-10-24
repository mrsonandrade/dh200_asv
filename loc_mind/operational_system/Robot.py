#! /usr/bin/env python3

'''
@author: Emerson Martins de Andrade
COPPE/UFRJ     2025
'''

import ps.behaviors as pb
import ps.utils as pu
import numpy as np
import time
from threading import Thread
import math
from datetime import timezone 
import datetime


class Robot(Thread, object):

    def __init__(self, **kwargs):
        Thread.__init__(self)

        global Communication, Motion, Position, Orientation, Sensors
        self.robot_id = kwargs.pop('RobotID')
        Communication = kwargs.pop('Communication')
        Motion = kwargs.pop('Motion')
        Position = kwargs.pop('Position')
        Orientation = kwargs.pop('Orientation')
        Sensors = kwargs.pop('Sensors')

        self.reference_lat_lng = [-22.863593, -43.215014] # LabOceano Pier

        self.full_verbose = True
        self.save_log = True
        
        self.state_dict = {"robot_id": None, 
                        "x": None,
                        "y": None,
                        "yaw": None,
                        "time": None,
                        "lat": None,
                        "lng": None,
                        "time_gps": None,
                        "ph": None,
                        "conductivity": None,
                        "temperature": None}
                
        self.motors_on = 1
        self.motors_power = 0 # from 0 to 100%
        self.yaw_constant = 0.33
        self.waypoints_lat_lng = []
        self.waypoints = []
        self.radius_of_achieved_waypoint = 10.0 # meters
        self.angle_of_achieved_heading = 5.0 # degrees
        
        self.current_goal = 0

        # navigation modes
        self.start_waypoints = 0

        # swarm behavior settings
        self.swarm_centroid_as_position = 0
        
        self.activate_target = 1
        self.swarm_target = np.asarray([113.83, -334.25, 0.])
        self.swarm_target_lat_lng = [-22.866599, -43.213903]
        
        self.activate_repulsion = 0
        self.repulsion_alpha = 5.0
        self.repulsion_d = 2
        
        self.activate_aggregation = 0

        self.state_history = {
            'time': [],
            'time_gps': [],
            'x': [], # meters
            'y': [], # meters
            'reference_lat_lng': [],
            'lat': [],
            'lng': [],
            'yaw': [], # degrees
            'x_est': []
            }
        
        self.max_duty_cycle_motors = 100.0
        self.right_motor = 0
        self.left_motor = 0
        
        self.robots = {}
        self.neighbours = {}

        self.past_time_list = []

        self.timer_update = time.process_time() # starts the timer on
        self.timer_broadcast = time.process_time()
        self.timer_if_error = time.process_time()
        self.timer_log = time.process_time()

        self.heading_error_old = None
        self.heading_error_derivative = 0.0
    
        # Earth's radius in meters
        self.EARTH_RADIUS = 6378137.
        
        self.log_filename = datetime.datetime.now().strftime("log/log_%H%M%S_%d%m%Y.csv")
        self.f = open(self.log_filename, "a")
        self.f.close()

        self._start_thread('main')

    def _start_thread(self, name):
        if name == 'main':
            self.main_loop = Thread(target=self._update)
            self.main_loop.daemon = True
            self.main_loop.start()
            
    def log_state(self, state):
        self.f = open(self.log_filename, "a")
        self.f.write(str(state)+'\n')
        self.f.close()
    
    def check_poses(self, data):
        for key_id in data.keys():
            if key_id not in self.robots.keys():
                self.robots[key_id] = {}
            for key_info_type in data[key_id].keys():
                if key_info_type=="pose":
                    self.robots[key_id] = data[key_id][key_info_type]
                    
    def add_this_robot_to_data(self):
        if self.robot_id not in self.robots.keys():
            self.robots[self.robot_id] = {}

    def haversine_to_xy(self, lat1, lon1, lat2, lon2):
        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Differences in coordinates
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        # Haversine formula for distance
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = self.EARTH_RADIUS * c  # Distance between points in meters

        # Calculate x and y coordinates in meters
        x = self.EARTH_RADIUS * dlon * math.cos((lat1 + lat2) / 2)
        y = self.EARTH_RADIUS * dlat

        return x, y
    
    def check_commands(self, data):
        
        try: self.motors_on = data[self.robot_id]['motors_on']
        except: pass
        try: self.motors_power = data[self.robot_id]['motors_power']
        except: pass
        
        try: self.waypoints_lat_lng = data[self.robot_id]['wpts_lat_lng']
        except: pass
        try: self.start_waypoints = data[self.robot_id]['start_waypoints']
        except: pass
        
        try: self.activate_target = data[self.robot_id]['activate_target']
        except: pass
        try: self.activate_repulsion = data[self.robot_id]['activate_repulsion']
        except: pass
        try: self.activate_aggregation = data[self.robot_id]['activate_aggregation']
        except: pass

        try: self.swarm_centroid_as_position = data[self.robot_id]['swarm_centroid_as_position']
        except: pass
        
        try: self.repulsion_alpha = data[self.robot_id]['repulsion_alpha']
        except: pass
        
        try: self.repulsion_d = data[self.robot_id]['repulsion_d']
        except: pass
        
        try: self.yaw_constant = data[self.robot_id]['yaw_constant']
        except: pass

    def adjust_heading(self, heading_error):
        
        if heading_error<-self.angle_of_achieved_heading: # turn right
            Motion.move(0.75*self.motors_power, -0.75*self.motors_power, 2)
            print('turnning right')

        elif heading_error>self.angle_of_achieved_heading: # turn left
            Motion.move(-0.75*self.motors_power, 0.75*self.motors_power, 2)
            print('turnning left')

        else:
            Motion.move(self.motors_power, self.motors_power)
            print('moving forward')

    def _update(self):

        while True:

            dtime = time.process_time() - self.timer_update # time elapsed in seconds
            if (dtime>0.1):
                ##############################################
                # SENSE
                ##############################################
                # get its own pose/state/sensors
                self.check_poses(Communication.data.copy())
                self.add_this_robot_to_data()
                self.check_commands(Communication.data.copy())
                
                if Communication.manual_command!="":
                    cmd = Communication.manual_command.split(",")
                    if cmd[0]==self.robot_id and cmd[1]=="0": # STOP
                        Motion.stop()
                    elif cmd[0]==self.robot_id and cmd[1]=="1": # FORWARD
                        Motion.move(self.motors_power, self.motors_power) # left_motor, right_motor
                    elif cmd[0]==self.robot_id and cmd[1]=="2": # BACKWARD
                        Motion.move(-self.motors_power, -self.motors_power)
                    elif cmd[0]==self.robot_id and cmd[1]=="3": # LEFT
                        Motion.move(-self.motors_power, self.motors_power)
                    elif cmd[0]==self.robot_id and cmd[1]=="4": # RIGHT
                        Motion.move(self.motors_power, -self.motors_power)

                ##############################################
                # THINK
                ##############################################
                # WHAT are the "objectives" [target, aggregation, etc.]

                try:
                    self.robots[self.robot_id] = [0,1,90,100]
                    
                    robot_names = list(self.robots.copy().keys())
                
                    r_i_index = robot_names.index(self.robot_id)

                    robot_positions = np.asarray([[self.robots[r][0],
                                                   self.robots[r][1], 0] for r in robot_names])
                                                   
                except Exception as e:
                    print(e)

                robot_x, robot_y = self.haversine_to_xy(self.reference_lat_lng[0],
                                                        self.reference_lat_lng[1],
                                                        Position.lat,
                                                        Position.lng)
                
                self.state_history['lat'].append(Position.lat)
                self.state_history['lng'].append(Position.lng)
                self.state_history['reference_lat_lng'].append(self.reference_lat_lng)
                
                self.state_history['x'].append(robot_x)
                self.state_history['y'].append(robot_y)
                self.state_history['yaw'].append(Orientation.heading_angle) # degrees

                # Updating the current date and time 
                dt = datetime.datetime.now(timezone.utc) 
                utc_time = dt.replace(tzinfo=timezone.utc) 
                utc_timestamp = utc_time.timestamp()
                self.state_history['time'].append(utc_timestamp)
                self.state_history['time_gps'].append(Position.date_time)
                
                #updating self.robots with this robot data
                self.robots[self.robot_id] = [self.state_history['x'],
                                             self.state_history['y'],
                                             self.state_history['yaw'],
                                             self.state_history['time']] # [x,y,yaw,t]
                
                # updating target xy
                try:
                    if len(self.waypoints_lat_lng)>0:
                        waypoints_meters = []
                        for lat_lng_i in self.waypoints_lat_lng:
                            x_m,y_m = self.haversine_to_xy(lat_lng_i[1],
                                                           lat_lng_i[0],
                                                           Position.lat,
                                                           Position.lng)
                                                           
                            waypoints_meters.append([-x_m,-y_m])
                        self.waypoints = np.asarray(waypoints_meters)
                        self.swarm_target = np.asarray([self.waypoints[self.current_goal][0],
                                                        self.waypoints[self.current_goal][1],
                                                        0.])
                        robot_positions[r_i_index][0] = 0
                        robot_positions[r_i_index][1] = 0
                                                        
                except Exception as e:
                    print(e)
                    
                if self.swarm_centroid_as_position:
                    ref_pos = pu.swarm_centroid(robot_positions)
                else:
                    ref_pos = robot_positions[r_i_index]
                    
                distance = np.linalg.norm(self.swarm_target - ref_pos)
                    
                ##############################################
                # ACT
                ##############################################
                # apply the decision
                try:
                    if self.start_waypoints:
                        print('current goal', self.current_goal, 'distance2waypoint', distance)#, 'radius_allowed',self.radius_of_achieved_waypoint)
                        if distance < self.radius_of_achieved_waypoint:
                            self.current_goal = (1+self.current_goal) % len(self.waypoints_lat_lng)
                            self.swarm_target = np.asarray([self.waypoints[self.current_goal][0],
                                                            self.waypoints[self.current_goal][1],
                                                            0.])
                        
                        if distance >= self.radius_of_achieved_waypoint:
                            r_i = robot_positions[r_i_index]
                            r_j = np.delete(robot_positions, np.array([r_i_index]), axis=0)
                            
                            if self.activate_target: r_t = pb.target(r_i, self.swarm_target) # TARGET
                            else: r_t = [0,0]
                            if self.activate_repulsion: r_r = pb.repulsion(r_i, r_j, self.repulsion_alpha, self.repulsion_d) # REPULSION
                            else: r_r = [0,0]
                            if self.activate_aggregation: r_a = pb.aggregation(r_i, r_j) # AGGREGATION
                            else: r_a = [0,0]
                            
                            r_final = [r_t[0]+r_r[0]+r_a[0],
                                       r_t[1]+r_r[1]+r_a[1]]
                                                        
                            r_final_rad = pu.ensure_negative_pi_to_pi(np.arctan2(r_final[1], r_final[0]))
                            this_robot_heading_rad = pu.ensure_negative_pi_to_pi(np.radians(90.0-self.state_history['yaw'][-1] % 360))                            
                            vel_angular_z = ((np.degrees(r_final_rad) - np.degrees(this_robot_heading_rad)) + 180) % 360 - 180
                                                                        
                            print(np.degrees(r_final_rad), np.degrees(this_robot_heading_rad))
                            print('target', self.swarm_target,'heading',self.state_history['yaw'][-1],'tgt_heading',vel_angular_z)
                            
                            if self.heading_error_old==None:
                                self.heading_error_old = vel_angular_z
                            self.heading_error_derivative = pu.ensure_negative_pi_to_pi(vel_angular_z - self.heading_error_old)/dtime

                            self.adjust_heading(vel_angular_z)

                            self.heading_error_old = vel_angular_z
                        
                except Exception as e:
                    print(e)

                self.timer_update = time.process_time()

            dtime_broadcast = time.process_time() - self.timer_broadcast # time elapsed in seconds
            if (dtime_broadcast>2.0):
                try:
                    # broadcast pose
                    msg_string = "robot_id,%s,pose,%.2f,%.2f,%.2f,%.6f" % (self.robot_id,
                                                                             self.state_history['x'][-1],
                                                                             self.state_history['y'][-1],
                                                                             self.state_history['yaw'][-1],
                                                                             self.state_history['time'][-1])
                                                                

                    if self.full_verbose==True:
                        msg_string = "robot_id,%s,pose,%.2f,%.2f,%.2f,%.6f,lat,%.6f,lng,%.6f,datetimegps,%s" % (self.robot_id,
                                                                                 self.state_history['x'][-1],
                                                                                 self.state_history['y'][-1],
                                                                                 self.state_history['yaw'][-1],
                                                                                 self.state_history['time'][-1],
                                                                                 self.state_history['lat'][-1],
                                                                                 self.state_history['lng'][-1],
                                                                                 self.state_history['time_gps'][-1])
                        Communication.send_message(msg_string)
                
                except Exception as e:
                    print(e)
                self.timer_broadcast = time.process_time()
            
            dtime_log = time.process_time() - self.timer_log # time elapsed in seconds
            if (dtime_log>2.0):
                if self.save_log==True:
                    self.state_dict["robot_id"] = self.robot_id
                    self.state_dict["x"] = self.state_history['x'][-1]
                    self.state_dict["y"] = self.state_history['y'][-1]
                    self.state_dict["yaw"] = self.state_history['yaw'][-1]
                    self.state_dict["time"] = self.state_history['time'][-1]
                    self.state_dict["lat"] = self.state_history['lat'][-1]
                    self.state_dict["lng"] = self.state_history['lng'][-1]
                    self.state_dict["time_gps"] = self.state_history['time_gps'][-1]
                    self.state_dict["ph"] = Sensors.data['ph']
                    self.state_dict["conductivity"] = Sensors.data['conductivity']
                    self.state_dict["temperature"] = Sensors.data['temperature']
                    #print(self.state_dict)
                    self.log_state(self.state_dict)
                self.timer_log = time.process_time()
                    
