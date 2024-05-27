# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""
import logging
import time
import math
import threading
from matplotlib import pyplot as plt
#from threading import Timer


import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

import dijkstra_algorithm as dijk
import numpy

uri = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E720')
lock = threading.Lock()

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self.states = {}
        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        self._lg_stab.add_variable('range.front')
        self._lg_stab.add_variable('range.back')
        self._lg_stab.add_variable('range.left')
        self._lg_stab.add_variable('range.right')
        self._lg_stab.add_variable('range.up')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        #print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            #print(f'{name}: {value:3.3f} ', end='')
            self.states[name] = value
        #print()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

not_detected = [True, True, True, True]
start_looking_for_obstacles = False

## HYPERPARAM ##
height_desired = 0.4
obstacle_detection = 400
map_x = 5
map_y = 2
length = 0.1
x_start = 0.2
y_start = 1
rows = int(map_x / length)
columns = int(map_y / length)
visited_nodes = []
init_goal_x = 4
init_goal_y = 1
search_mode_threshold = 3.75
multiplier = 1
go_back = False
rows_to_remove = 1
columns_to_remove = 1
## 
#dijk.print_msg()
graph, node_points, connections, best_path_edges, best_path_nodes, fig, node_ids, goal = dijk.generate_dijkstra(rows, columns, length, 0, 0, x_start, y_start, init_goal_x, init_goal_y)
print("Node ids: " + str(node_ids))
visual = connections
print("Best path nodes " + str(best_path_nodes))
#fig = dijk.visualisations(node_points, connections, best_path_nodes, best_path_edges)

def timer():
    global not_detected
    global graph, node_points, connections, best_path_edges, best_path_nodes, fig
    while True:
        #print("Selma dead and thread " + str(not_detected))
        for i in range(len(not_detected)):
            if(not not_detected[i]):
                print("OBSTACLLLEEEEEEEE")
                #connections = dijk.remove(graph, 6)
                #if(connections != None):
                    #plt.close(fig)
                    #best_path_edges, best_path_nodes = dijk.get_best_path(graph, node_points, start, goal)
                #dijk.draw_map(fig)
                time.sleep(1)
                print("done")
                with lock:
                    not_detected[i] = True

        time.sleep(0.1)   # 3 seconds.

def idle (cf, height, length):
    for y in range(length):
        cf.commander.send_hover_setpoint(0, 0, 0, height)
        time.sleep(0.1)

def idle_at_point(cf, height, length, point):
    for y in range(length):
        cf.commander.send_position_setpoint(point[0], point[1], height, 0)
        time.sleep(0.1)
        
def check_if_arrived(pos_bot, pos_goal, threshold):
    if(math.dist(pos_bot, pos_goal) < threshold):
        return True
    return False

def takeoff(cf):
    for y in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, y/25)
        time.sleep(0.1)
    idle(cf, height_desired, 20)

def detect_obstacle(sensor, threshold):
    if sensor < threshold:
        return True
    
def set_start_looking_for_obstacles():
    global start_looking_for_obstacles
    start_looking_for_obstacles = True

def sensor_detected_obstacle(range_front, range_left, range_right, range_back):
    #front, left, right, back
    sensor = [False, False, False, False]
 
    if detect_obstacle(range_front, obstacle_detection):
        sensor[0] = True
    if detect_obstacle(range_left, obstacle_detection):
        sensor[1] = True
    if detect_obstacle(range_right, obstacle_detection):
        sensor[2] = True
    if detect_obstacle(range_back, obstacle_detection):
        sensor[3] = True
    return sensor

def get_search_nodes(nodes_points):
    search_nodes = []
    for node in nodes_points:
        if node[0] >= search_mode_threshold:
            search_nodes.append(node)
    return search_nodes

def get_closest_node(node_points, x, y, height):
    point = numpy.array((x,y, height))
    distances = numpy.linalg.norm(node_points-point, axis=1)
    node = numpy.argmin(distances)
    return node

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    le = LoggingExample(uri)
    cf = le._cf

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)
    #cf.param.set_value('kalman.initialX', '0')
    #time.sleep(2)
    #cf.param.set_value('kalman.initialY', '0.98')
    #time.sleep(2)
    print("Selma is a baddie")
    #time.sleep(5)
    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    state  = 0
    seq_index = 0
    sequence = [
    (0.5, 0, height_desired),
    (0.5, 0.5, height_desired),
    (0, 0.5, height_desired),
    (0.0, 0.0, height_desired),
    ]
    x_past = 0
    y_past = 0
    dijk.draw_map(fig)
    converted_all_nodes = dijk.conversion(node_points, height_desired)
    converted_nodes = dijk.conversion(best_path_nodes, height_desired)
    search_nodes = get_search_nodes(converted_all_nodes)
    print("SEARCH NODES " + str(search_nodes))
    print(sequence)
    sequence = converted_nodes
    print(sequence)
    print(graph)
    t = threading.Thread(target=timer)
    t.start()
    t2 = threading.Timer(5, set_start_looking_for_obstacles)
    t2.start()
    print("Going in")
    last_index = 0
    visited_search_nodes = []
    while le.is_connected:
        x = le.states['stateEstimate.x'] 
        y = le.states['stateEstimate.y'] 
        z = le.states['stateEstimate.z']
        range_up = le.states['range.up']
        range_front = le.states['range.front']
        range_right = le.states['range.right']
        range_left = le.states['range.left']
        range_back = le.states['range.back']
        time.sleep(0.01)

        def obstacle_avoidance():
            global visual, best_path_edges, node_ids, seq_index
            obstacles_detected = sensor_detected_obstacle(range_front, range_left, range_right, range_back)
            #print(obstacles_detected)
            true_indices = [i for i, x in enumerate(obstacles_detected) if x]
            #true_not_detected = [i for i, x in enumerate(not_detected) if x] 
            if(len(true_indices) > 0 and start_looking_for_obstacles):
                #print("Detected")
                for i in range(len(true_indices)):
                    #connections = None
                    print(i)
                    print(true_indices)
                    print(not_detected)
                    if(not_detected[true_indices[i]]):
                        #with lock:
                            #not_detected[true_indices[i]] = False
                        if(true_indices[i] == 0):
                            obst_x = x + range_front/1000
                            obst_y = y
                        elif(true_indices[i] == 1):
                            obst_x = x
                            obst_y = y + range_left/1000
                        elif(true_indices[i] == 2):
                            print("HERE!")
                            obst_x = x
                            obst_y = y - range_right/1000
                        elif(true_indices[i] == 3):
                            obst_x = x - range_back/1000
                            obst_y = y
                        obstacle = numpy.array((obst_x, obst_y, height_desired))
                        print("obstacle_value : " + str(obstacle))

                        distances = numpy.linalg.norm(converted_all_nodes-obstacle, axis=1)
                        min_index = numpy.argmin(distances)
                        print("Node to remove is ", + min_index)
                        connections = dijk.remove(graph, min_index) 
                        for i in range(rows_to_remove):
                            row_chooser_pos = i * columns
                            row_chooser_neg = -i *columns
                            for j in range(columns_to_remove):
                                current_node_pos = min_index + row_chooser_pos + j
                                current_node_neg = min_index + row_chooser_pos - j
                                if current_node_pos in graph: 
                                    connections = dijk.remove(graph, current_node_pos)
                                    converted_all_nodes[current_node_pos] = (1000, 1000, 1000)
                                if current_node_neg in graph: 
                                    connections = dijk.remove(graph, current_node_neg)
                                    converted_all_nodes[current_node_neg] = (1000, 1000, 1000)
                            for l in range(columns_to_remove):
                                current_node_pos = min_index + row_chooser_neg + l
                                current_node_neg = min_index + row_chooser_neg - l
                                if current_node_pos in graph: 
                                    connections = dijk.remove(graph, current_node_pos)
                                    converted_all_nodes[current_node_pos] = (1000, 1000, 1000)
                                if current_node_neg in graph: 
                                    connections = dijk.remove(graph, current_node_neg)
                                    converted_all_nodes[current_node_neg] = (1000, 1000, 1000)
                        
                        print("removed it")
                        if(connections != None):
                            visual = connections
                            potential_start = get_closest_node(converted_all_nodes, x, y, height_desired)
                            if potential_start in graph: 
                                start_index_bot = potential_start
                            else:
                                for i in range(len(visited_nodes)):
                                    if (visited_nodes[len(visited_nodes) - 1-i] in graph):
                                        start_index_bot = visited_nodes[len(visited_nodes) - 1-i]
                                        break
                                    else:
                                        ("Oh no")
                                        start_index_bot = 0
                            print("start_index: " + str(start_index_bot))
                            print("got fams position")
                            cf.commander.send_hover_setpoint(0, 0, 0, height_desired)
                            print("dijkstra")
                            best_path_edges, best_path_nodes, node_ids = dijk.get_best_path(graph, node_points, start_index_bot, goal, 0)
                            print(best_path_nodes)
                            seq_index = 0
                            print("finna convert")
                            converted_nodes = dijk.conversion(best_path_nodes, height_desired)
                            print(converted_nodes)
                            print("Node ids: " + str(node_ids))
                            sequence = converted_nodes
                            return sequence
            return None

        if state == 0:
            print("Take off")
            takeoff(cf)
            state = 1
        if state == 1:
            #print("Getting to the search area")
            cf.commander.send_position_setpoint(sequence[seq_index][0], sequence[seq_index][1], sequence[seq_index][2], 0)
            pos_bot = [x, y]
            pos_goal = [sequence[seq_index][0], sequence[seq_index][1]]
            if(check_if_arrived(pos_bot, pos_goal, 0.05)):
                #idle_at_point(cf, height_desired, 5, pos_goal)
                visited_nodes.append(node_ids[seq_index])
                last_index = node_ids[seq_index]
                seq_index += 1
                if (x > search_mode_threshold- x_start):
                    seq_index =  0
                    state = 2
            new_sequence = obstacle_avoidance()
            if new_sequence != None:
                sequence = new_sequence
        if(state == 2):
            print("Search Mode")
            start_search = get_closest_node(converted_all_nodes, x, y, height_desired)
            #closest_in_search = get_closest_node(search_nodes, x, y, height_desired) + 1
            print("start : " + str(start_search))
            for i in range(1, len(search_nodes), 2):
                i = i * multiplier
                if start_search+i in graph:
                    goal_search = start_search+i
                    break
            #goal_search = closest_in_search 
            #print(goal_search)
            #if(go_back):
             #   goal_search = search_nodes
            best_path_edges, best_path_nodes, node_ids = dijk.get_best_path(graph, node_points, start_search, goal_search, 0)
            converted_nodes = dijk.conversion(best_path_nodes, height_desired)
            sequence = converted_nodes
            seq_index = 0
            counter = 0
            state = 4
        if (state == 4):
            #print("Searching")
            cf.commander.send_position_setpoint(sequence[seq_index][0], sequence[seq_index][1], sequence[seq_index][2], 0)
            pos_bot = [x, y]
            pos_goal = [sequence[seq_index][0], sequence[seq_index][1]]
            if(check_if_arrived(pos_bot, pos_goal, 0.05)):
                print(node_ids)
                print(seq_index)
                print(sequence)
                visited_nodes.append(node_ids[seq_index])
                last_index = node_ids[seq_index]
                seq_index += 1
                converted_nodes = dijk.conversion(best_path_nodes, height_desired)
                if (seq_index > len(sequence) - 1):
                    seq_index =  0
                    start_search = get_closest_node(converted_all_nodes, x, y, height_desired)
                    print(start_search)
                    #print("search nodes"search_nodes)
                    #goal_search = get_closest_node(converted_all_nodes, search_nodes[counter][0], search_nodes[counter][1], height_desired)
                    #print(goal_search)
                    count_search = 0
                    for i in range(1, len(search_nodes), 3):
                        i = i * multiplier
                        if start_search+i in graph:
                            goal_search = start_search+i
                            if(goal_search == start_search):
                                count_search += 1
                            else: count_search = 0
                            if(count_search > 4): go_back = True
                            break
                    print("Goal search : " + str(goal_search))
                    best_path_edges, best_path_nodes, node_ids = dijk.get_best_path(graph, node_points, start_search, goal_search, search_nodes[0])
                    converted_nodes = dijk.conversion(best_path_nodes, height_desired)
                    sequence = converted_nodes
                    #counter += 1
                    print("sequence : " + str(sequence))
                    print("Search nodes lenght : " + str(len(search_nodes)))
                    print("counter " + str(counter))
                    if (start_search == goal_search):
                        multiplier = -1
                        go_back = True
                        state = 2
                    #if (go_back):
                        #multiplier = 1
                        #go_back = False
                        #state = 2
            goal = goal_search
            new_sequence = obstacle_avoidance()
            if new_sequence != None:
                sequence = new_sequence
            if (z < height_desired - 0.05):
                print("In here")
                #state = 5
        if state == 5:
            print("Landing")
            for y in range(5, -1, -1):
                cf.commander.send_hover_setpoint(0, 0, 0, y/25)
                time.sleep(0.1)
            cf.commander.send_stop_setpoint()
            plt.close()
            #print(visual)
            #print(best_path_nodes)
            fig = dijk.visualisations(node_points, visual, best_path_nodes, best_path_edges)
            dijk.draw_map(fig)
            #time.sleep(5)
            print("finito")
            break
