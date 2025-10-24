#! /usr/bin/env python3

'''
@author: Emerson Andrade
COPPE/UFRJ     2025
'''

import numpy as np
import json

def process_data(data):
    """
    data_example = ['robot_id','01',
                    'pose','100','80','35','12.46',
                    'command',target_xy','100','80']
    """
    
    data_dict = {}
    manual_command = "-1"

    for i in range(len(data)):

        ##############################################################
        # GET ROBOT IDENDIFICATION
        ##############################################################

        if data[i]=='robot_id':
            robot_id = data[i+1]
            if robot_id not in data_dict.keys():
                data_dict[robot_id] = {}

        ##############################################################
        # GET ROBOT STATE
        ##############################################################

        if data[i]=='pose':
            x = float(data[i+1])
            y = float(data[i+2])
            yaw = float(data[i+3])
            t = round(float(data[i+4]), 2) # time in seconds
            try:
                data_dict[robot_id]['pose'] = [x,y,yaw,t]
            except Exception as e:
                print(e)

        ##############################################################
        # PROCESS ROBOT COMMANDS
        ##############################################################
        
        if data[i]=='rc':
            manual_command = robot_id+","+data[i+1]

        if data[i]=='command':
            parameter = data[i+1]

            ##############################################################
            # Motors
            ##############################################################
            if parameter=='motors_on':
                try: data_dict[robot_id]['motors_on'] = int(data[i+2])
                except Exception as e: print(e)
            if parameter=='motors_power':
                try: data_dict[robot_id]['motors_power'] = int(data[i+2])
                except Exception as e: print(e)
                
            ##############################################################
            # Waypoint navigation
            ##############################################################
            if parameter=='wpts':
                try: data_dict[robot_id]['wpts_lat_lng'] = np.array(json.loads(','.join(data[i+2:])))/1000000.
                except Exception as e: print(e)
            if parameter=='swpts':
                try: data_dict[robot_id]['start_waypoints'] = int(data[i+2])
                except Exception as e: print(e)
                
            ##############################################################
            # Activate/Deactivate algorithms
            ##############################################################
            if parameter=='activate_target':
                try: data_dict[robot_id]['activate_target'] = int(data[i+2])
                except Exception as e: print(e)
            if parameter=='activate_repulsion':
                try: data_dict[robot_id]['activate_repulsion'] = int(data[i+2])
                except Exception as e: print(e)
            if parameter=='activate_aggregation':
                try: data_dict[robot_id]['activate_aggregation'] = int(data[i+2])
                except Exception as e: print(e)

            ##############################################################
            # Target algorithm
            ##############################################################
            if parameter=='target_xy':
                x = float(data[i+2])
                y = float(data[i+3])
                try: data_dict[robot_id]['target_xy'] = [x,y]
                except Exception as e: print(e)

            ##############################################################
            # Repulsion algorithm
            ##############################################################
            if parameter=='repulsion_alpha':
                try: data_dict[robot_id]['repulsion_alpha'] = float(data[i+2])
                except Exception as e: print(e)

            if parameter=='repulsion_d':
                try: data_dict[robot_id]['repulsion_d'] = int(data[i+2])
                except Exception as e: print(e)

            ##############################################################
            # Others
            ##############################################################
            if parameter=='swarm_centroid_as_position':
                try: data_dict[robot_id]['swarm_centroid_as_position'] = int(data[i+2])
                except Exception as e: print(e)

            if parameter=='yaw_constant':
                try: data_dict[robot_id]['yaw_constant'] = float(data[i+2])
                except Exception as e: print(e)
                
    return data_dict, manual_command
