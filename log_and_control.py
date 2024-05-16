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
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E720')

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
        t = Timer(50, self._cf.close_link)
        t.start()

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

def PD_controller(x_past, x_current, y_past, y_current, xgoal, y_goal, threshold = 0.075):
    Kp = 0.5
    Kd = 0
    reached_x = False
    reached_y = False

    error_x = xgoal - x_current
    derivative_x = x_current - x_past

    error_y = y_goal - y_current
    derivative_y = y_current - y_past

    v_x = Kp * error_x + Kd * derivative_x
    v_y = Kp * error_y + Kd * derivative_y

    if(abs(error_x) < threshold): reached_x = True
    if(abs(error_y) < threshold): reached_y = True  
    return v_x, v_y, error_x, error_y, reached_x, reached_y

def idle (cf, height, length):
    for y in range(length):
        cf.commander.send_hover_setpoint(0, 0, 0, height)
        time.sleep(0.1)

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = LoggingExample(uri)
    cf = le._cf

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)
    print("hi")
    time.sleep(5)
    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    state  = 0
    seq_index = 0
    sequence = [
    (0.25, 0.25),
    (0.25, 0.0),
    #(0.25, 0.25),
    #(0.0, 0.25),
    #(0.0, 0.0)
    ]
    x_past = 0
    y_past = 0
    while le.is_connected:
        x = le.states['stateEstimate.x']
        y = le.states['stateEstimate.y']
        z = le.states['stateEstimate.z']
        time.sleep(0.01)
        #print(le.states)
        #print(le._lg_stab.variables[0].value)
        if state == 0:
            for y in range(10):
                cf.commander.send_hover_setpoint(0, 0, 0, y/25)
                time.sleep(0.1)
            state = 1
        if state == 1:
            idle(cf, 0.4, 20)
            state = 2
        if state == 2:
            cf.commander.send_position_setpoint(0.5, 0, 0.4, 0)
            if(le.states['stateEstimate.x'] > 0.5):
                idle(cf, 0.4, 20)
                state = 3
        if state == 3:
            cf.commander.send_position_setpoint(0, 0, 0.4, 0)
            if(le.states['stateEstimate.x'] < 0):
                idle(cf, 0.4, 20)
                state = 4
        if state == 4:
            for y in range(10, -1, -1):
                cf.commander.send_hover_setpoint(0, 0, 0, y/25)
                time.sleep(0.1)
            cf.commander.send_stop_setpoint()
            break

          #  for pos in sequence:
              #  for _ in range(50):
               #     cf.commander.send_hover_setpoint(pos[0], pos[1], pos[2], pos[3])
                 #   time.sleep(0.1)

            #state = 2
        
        if (state == 99):
            x_goal = 0.5
            y_goal = 0
           # print("x_goal : " + str(x_goal) + "    y : " + str(y_goal))
            v_x, v_y, error_x, error_y, reached_x, reached_y = PD_controller(x_past, x, y_past, y, x_goal, y_goal)
            #print("X reached : " + str(reached_x) + " Y reached : " + str(reached_y))
            #print("Vx : " + str(v_x) + "Vy : " + str(v_y))
            time.sleep(0.1)
            if(reached_x):
                print("Reached!")
                idle(cf, 0.4, 20)
                #state = 4
            #vy, error_y, reached_y = PD_controller(y_past, le.states['stateEstimate.y'], sequence[seq_index][1])
            #if (reached_x and reached_y):
                #print("Point " + str(sequence[seq_index]))
                #seq_index += 1
                #idle(cf, 0.4, 50)
                #print("seq index" + str(seq_index))
                #print("sequence " + str(len(sequence)))
                #print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                #if seq_index > len(sequence): state = 4
            cf.commander.send_position_setpoint(v_x, v_y, 0, 0.4)
           # print("vx : " + str(vx) + "     vy : " + str(vy))
            #cf.high_level_commander.go_to(1, 0, 0.4, 0, 5)
            #x_past = le.states['stateEstimate.x']
           # y_past = le.states['stateEstimate.y']
        print("x : " + str(le.states['stateEstimate.x']) + "   y : " + str(le.states['stateEstimate.y']))
        #if (state == 2):
           # pass
            #cf.commander.send_position_setpoint(0.5, 0, 0.4, 0)
            #cf.commander.send_position_setpoint(0, 0, 0.4, 0)
        #for _ in range(50):
        #    cf.commander.send_hover_setpoint(0.5, 0, 0, 0.4)
        #    time.sleep(0.1)
        
        #for _ in range(20):
         #   cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
         #   time.sleep(0.1)

        #x = cf.param.get_value('kalman.resetEstimation', '0')
