import logging
import time
import math
import numpy as np
import random
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E720')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

#========================== STATES VARIABLES =============================#

NORMAL, FOLLOWING_OUTLINE, STOPPING, SEARCHING_LANDING_PAD, PREPARE_TO_LAND, LANDING,\
TURNING_LEFT_90, MOVING_FORWARD, TURNING_180, SHIFTING_RIGHT, SWEEPING, TAKEOFF, RETURN_STARTING_ZONE,\
SEARCHING_TAKEOFF_PAD = range(14)

#======================= INITIAL STATE VARIABLE ==========================#

drone_state = NORMAL

#========================== GLOBAL VARIABLES =============================#

on_ground = True
height_desired = 0.3
startpos = None

yaw = 0.0  # Initialize yaw to zero
vx = 0.5  # Constant forward speed along x-axis
vy = 0.0  # No lateral movement initially

sensor_data = {
'x_global': 0.0,
'y_global': 0.0,
'z_global': 0.0,
'range_down': 0.0,
'range_front': 0.0,
'range_right': 0.0,
'range_left': 0.0,
'range_back': 0.0,
'yaw': 0.0
}

critical_dist = 0.15  # Critical distance for obstacle detection
front_dist = 0.4
landing_pad_height_threshold = 0.9

avoidance_direction = 'right'  # Default avoidance direction
avoiding_obstacle = False

post_obstacle_counter = 0
post_obstacle_time = 10  # Nombre de cycles à maintenir vy après la détection

safety_margin = 0.25
x_lenght_arena = 5
y_lenght_arena = 3
middle_y = (0 + y_lenght_arena) / 2
middle_x = (0 + x_lenght_arena) / 2
max_speed = 0.5
landing_zone_start_position = 2/3 * x_lenght_arena
takeoff_zone_start_position = 1/3 * x_lenght_arena

angle_reached = False
saving_yaw = True

actual_yaw = None
target_yaw = None

turn_start_time = None
turn_duration = 3.5  # Durée de la rotation en secondes

shift_start_time = None
shift_duration = 0.18  # Durée de la rotation en secondes

sweep_direction = None  # 'forward' pour avancer, 'backward' pour reculer
sweep_start_time = None
sweep_duration = 5  # Durée de chaque phase de sweep avant de shift

waiting_for_landing_zone = True
waiting_for_takeoff_zone = False
return_start = False
near_x_limit = False

y_speed = 0
random_value = 0

outerbound = True
innerbound = False

x_landing_pad = 0
y_landing_pad = 0
error_x = 0
error_y = 0

cycle_count = 0
update_interval = 500  # Nombre de cycles entre chaque vérification de mouvement
last_positions = [0, 0]
min_movement_threshold = 0.1
    

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
        #t = Timer(100, self._cf.close_link)
        #t.start()

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


def get_command(sensor_data):
    global on_ground, startpos, yaw, vx, vy, height_desired, avoidance_direction, drone_state, saving_yaw, actual_yaw,\
        target_yaw, turn_start_time, waiting_for_landing_zone, waiting_for_takeoff_zone, previous_state, avoiding_obstacle, last_time_called, x_landing_pad, y_landing_pad,\
        error_x, error_y, return_start, cycle_count, last_positions, update_interval, min_movement_threshold

    # Initialisation de la position de départ et décollage contrôlé
    if startpos is None:
        startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']]
    
    if on_ground and sensor_data['range_down'] < 0.49:
        controlled_takeoff(cf)
        return [0.0, 0.0, height_desired, 0.0]
    else:
        on_ground = False    
    
            
    # Update orientation and alignment
    current_yaw = sensor_data['yaw']
    if not (-0.1 <= current_yaw <= 0.1) and waiting_for_landing_zone :
        yaw_correction = realign_to_x_axis(current_yaw, 0)
        vx, vy, yaw = 0, 0, yaw_correction
    else:
        vx, vy, yaw = max_speed, 0.0, 0.0

    # Navigation and obstacle avoidance
    if drone_state == NORMAL:
            
        if sensor_data['range_front'] < front_dist:
            
            if sensor_data['y_global'] > middle_y:
                avoidance_direction = 'left'  # Si le drone est dans la moitié droite, contourner par la gauche
            else:
                avoidance_direction = 'right' # Si le drone est dans la moitié gauche, contourner par la droite
            drone_state = FOLLOWING_OUTLINE
            
        if sensor_data['range_left'] < critical_dist and sensor_data['range_right'] >= critical_dist:
            vx, vy = turn_right(sensor_data['range_left'])
        elif sensor_data['range_right'] < critical_dist and sensor_data['range_left'] >= critical_dist:
            vx, vy  = turn_left(sensor_data['range_right'])
        elif sensor_data['range_left'] < critical_dist + 0.1 and sensor_data['range_right'] < critical_dist + 0.1:
            vx, vy, yaw = stop()    

    elif drone_state == FOLLOWING_OUTLINE:
        vx, vy = follow_outline(sensor_data['range_front'], sensor_data['range_left'], sensor_data['range_right'], avoidance_direction)
        
    elif drone_state == PREPARE_TO_LAND:
    
        # Calculer le vecteur de déplacement vers les coordonnées enregistrées
        error_x = x_landing_pad - sensor_data['x_global']
        error_y = y_landing_pad - sensor_data['y_global']
        
        # Utiliser une méthode de contrôle simple pour se rapprocher des coordonnées
        vx = np.clip(0.1 * error_x, -max_speed, max_speed)
        vy = np.clip(0.1 * error_y, -max_speed, max_speed)

        if abs(error_x) < 0.01 and abs(error_y) < 0.01:
            drone_state = LANDING
        
    elif drone_state == LANDING:
        
        vx, vy = 0, 0
        height_desired = sensor_data['range_down'] - 0.1  # Lentement descendre pour atterrir
        if sensor_data['range_down'] <= 0.05:
            on_ground = True
            if return_start == False:
                drone_state = TAKEOFF  # Modifier ici pour passer à TAKEOFF
            vx, vy, yaw = 0, 0, 0  # Drone a atterri, arrêter tous les mouvements
            
    elif drone_state == STOPPING:
        vx, vy, yaw = stop()

    if sensor_data['x_global'] > landing_zone_start_position and waiting_for_landing_zone:
        drone_state = SEARCHING_LANDING_PAD
    
    if sensor_data['x_global'] < takeoff_zone_start_position-0.1 and waiting_for_takeoff_zone:
        drone_state = SEARCHING_TAKEOFF_PAD    

    if sensor_data['range_down'] < landing_pad_height_threshold and drone_state in [SEARCHING_LANDING_PAD, MOVING_FORWARD, SWEEPING]:
        
        x_landing_pad = sensor_data['x_global']
        y_landing_pad = sensor_data['y_global']
        drone_state = PREPARE_TO_LAND

    if drone_state == SEARCHING_LANDING_PAD:
        stop()  # Arrête le drone
        drone_state = TURNING_LEFT_90

    if drone_state == SEARCHING_TAKEOFF_PAD:
        stop()  # Arrête le drone
        if vx == 0:
            drone_state = TURNING_LEFT_90
        
    if drone_state == TURNING_LEFT_90:
        
        vx = 0
        vy = 0
            
        if return_start:
            angle_reached = turn_by_angle(sensor_data['yaw'], -np.pi/2)
        else:
            angle_reached = turn_by_angle(sensor_data['yaw'], np.pi/2)    
        
        # Appliquer la commande de yaw si l'angle n'est pas encore atteint.
        if not angle_reached:
            vx = 0
            vy = 0
            yaw = 1
        else:
            # Si l'angle cible est atteint, passer à l'état suivant
            drone_state = MOVING_FORWARD
            
        waiting_for_landing_zone = False
        waiting_for_takeoff_zone = False
        
    if drone_state == MOVING_FORWARD:
    
        if return_start:
            yaw = realign_to_y_axis(sensor_data['yaw'], -np.pi/2)
        else:
            yaw = realign_to_y_axis(sensor_data['yaw'], np.pi/2)    
    
        if sensor_data['range_front'] < front_dist:
            
            if sensor_data['x_global'] > middle_x:
                avoidance_direction = 'left'  # Si le drone est dans la moitié droite, contourner par la gauche
            else:
                avoidance_direction = 'right' # Si le drone est dans la moitié gauche, contourner par la droite
            
        else:    
            vx = max_speed
            drone_state = MOVING_FORWARD
            
            # Vérifier si le drone atteint la limite de la carte sur l'axe x
            if sensor_data['y_global'] >= y_lenght_arena - safety_margin:
                # Arrêter le drone en ajustant la vitesse à 0
                vx = 0

                # Passer à l'état suivant pour tourner de 180 degrés
                drone_state = TURNING_180
                previous_state = drone_state
                
            # Vous pourriez avoir une logique similaire pour la limite minimale si nécessaire
            elif sensor_data['y_global'] <= safety_margin:
                # Arrêter le drone en ajustant la vitesse à 0
                vx = 0

                # Passer à l'état suivant pour tourner de 180 degrés
                drone_state = TURNING_180

    if drone_state == TURNING_180:
        vx, vy, yaw = turning_180(sensor_data['yaw'])
        
    if drone_state == SHIFTING_RIGHT:
        vx, vy, yaw = shift()
    
    if  drone_state == SWEEPING: 
        previous_state = drone_state   
        vx, vy, yaw = handle_sweeping(sensor_data)
        
        # Update orientation and alignment
        yaw = realign_to_y_axis(sensor_data['yaw'], -np.pi/2)

    if drone_state in [MOVING_FORWARD, SWEEPING]:
        
        vx, vy = handle_obstacle_avoidance(sensor_data)
        if avoiding_obstacle:
            return [vx, vy, height_desired, yaw]
    
    if drone_state == TAKEOFF:
        
        if sensor_data['range_down'] < 0.49:
            height_desired = 1
            vx, vy, yaw = 0, 0, 0
            on_ground = False  # Important pour permettre le redécollage  
        if sensor_data['range_down'] > 0.93:
            drone_state = RETURN_STARTING_ZONE
    
    if drone_state == RETURN_STARTING_ZONE:
        
        target_yaw = np.pi  # Cible à l'opposé sur l'axe X
        current_yaw = sensor_data['yaw']
        yaw_correction = realign_to_x_axis(current_yaw, target_yaw)
        vx = 0
        vy = 0
        waiting_for_takeoff_zone = True
        return_start = True
        angle_reached = False 
        # Seuil pour considérer que le drone est correctement aligné
        if abs(yaw_correction) < 0.1:
            yaw = 0  # Arrêter la rotation
            drone_state = NORMAL  # Passer à l'état suivant
        else:
            yaw = yaw_correction  # Appliquer la correction de yaw
    
    
    cycle_count += 1
    
    if cycle_count - update_interval == 0:
        stuck = unstuck(sensor_data, last_positions[0], last_positions[1])
        if stuck:    
            vx, vy = random.randint(-100,100), random.randint(-2,2)  # Commandes pour débloquer le drone   
            print("Unstuck maneuver triggered.")
            
        # Mise à jour de la position pour la prochaine vérification
        last_positions[0] = sensor_data['x_global']
        last_positions[1] = sensor_data['y_global']
        cycle_count = 0
            
    # Manage boundary constraints
    vx, vy = check_boundaries(sensor_data, x_lenght_arena, y_lenght_arena, safety_margin, vx, vy)
    
    # print(f"State: {drone_state}, X: {sensor_data['x_global']}, Y: {sensor_data['y_global']}, Down: {sensor_data['range_down']}")
    # print(sensor_data['yaw'])
    #print(f"State: {drone_state}")
    
    state_names = {
    NORMAL: "NORMAL",
    FOLLOWING_OUTLINE: "FOLLOWING_OUTLINE",
    STOPPING: "STOPPING",
    SEARCHING_LANDING_PAD: "SEARCHING_LANDING_PAD",
    LANDING: "LANDING",
    TURNING_LEFT_90: "TURNING_LEFT_90",
    MOVING_FORWARD: "MOVING_FORWARD",
    TURNING_180: "TURNING_180",
    SHIFTING_RIGHT: "SHIFTING_RIGHT",
    SWEEPING: "SWEEPING",
    TAKEOFF: "TAKEOFF",
    RETURN_STARTING_ZONE: "RETURN_STARTING_ZONE",
    SEARCHING_TAKEOFF_PAD: "SEARCHING_TAKEOFF_PAD",
    PREPARE_TO_LAND: "PREPARE_TO_LAND"
    }
    #if return_start:
        #print(f"State: {state_names.get(drone_state, 'Unknown state')}")
    # print(error_x, error_y)
    # Assemble the control command
    control_command = [vx, vy, height_desired, yaw]

    return control_command

def controlled_takeoff(cf, target_height=1, steps=50):
    if on_ground:
        increment = target_height / steps
        for step in range(steps + 1):
            current_height = increment * step
            cf.commander.send_hover_setpoint(0, 0, 0, current_height)
            time.sleep(0.1)
        on_ground = False
        
def handle_obstacle_avoidance(sensor_data):
    global vx, vy, avoiding_obstacle

    if sensor_data['range_front'] < front_dist or sensor_data['range_left'] < critical_dist or sensor_data['range_right'] < critical_dist or sensor_data['range_back'] < front_dist:
        if sensor_data['range_front'] < front_dist or sensor_data['range_back'] < front_dist :
            # Logique simple d'évitement: arrêt ou changement de direction
            if return_start == False:
                vy = (-0.35 if sensor_data['x_global'] > (landing_zone_start_position + 5)/2 else 0.25)
            else:
                vy = (-0.35 if sensor_data['x_global'] > (takeoff_zone_start_position + 0)/2 else 0.25)
            vx = 0
        avoiding_obstacle = True
    else:
        avoiding_obstacle = False
    return vx, vy

def handle_sweeping(sensor_data):
    global drone_state, sweep_start_time, vx, vy, yaw, sweep_direction, sweep_duration, avoidance_direction,\
        near_x_limit, random_value, y_speed, outerbound, innerbound, return_start
    
            
    if sweep_direction is None:
        # Déterminer la direction initiale basée sur la position y actuelle
        if sensor_data['y_global'] <= 0.5:  # Proche de y=0
            sweep_direction = 'forward'
        else:
            sweep_direction = 'backward'
    
    # Gérer le mouvement basé sur la direction
    elif sweep_direction == 'forward':
        vx = -max_speed+0.3  # Avancer vers y=3
        if sensor_data['y_global'] >= 2.9:  # Près de la limite y=3
            sweep_direction = 'backward'  # Changer de direction après atteindre la limite
    else:
        vx = max_speed-0.3  # Reculer vers y=0
        if sensor_data['y_global'] <= 0.1:  # Près de la limite y=0
            sweep_direction = 'forward'  # Changer de direction après atteindre la limite
            
    if sensor_data['range_front'] < front_dist:
            
            if sensor_data['x_global'] > middle_x:
                avoidance_direction = 'left'  # Si le drone est dans la moitié droite, contourner par la gauche
            else:
                avoidance_direction = 'right' # Si le drone est dans la moitié gauche, contourner par la droite
    
    
    if return_start:
        
        if outerbound and sensor_data['x_global'] <= 0 + safety_margin:
            near_x_limit = True
            outerbound = False
            innerbound = True
        elif innerbound and sensor_data['x_global'] >= takeoff_zone_start_position + safety_margin:
            near_x_limit = False
            outerbound = True
            innerbound = False
            
        if random_value == 0:
            if random.random() < 0.25 and sensor_data['x_global'] < x_lenght_arena - safety_margin * 3 :
                random_value = random.randint(20, 50)  # Duration for high-speed maneuver
                if random_value < 35: 
                    y_speed = -1.5  # Higher speed
                else:
                    y_speed = 1.5    
            else:
                random_value = random.randint(20, 1000)  # Longer duration for normal speed
                y_speed = random.uniform(0.03, 0.04)  # Normal random speed
        else:
            random_value -= 1  # Decrement after setting the speed

        vy = -y_speed if not near_x_limit else y_speed

        
    else:
        
        if outerbound and sensor_data['x_global'] >= x_lenght_arena - safety_margin:
            near_x_limit = True
            outerbound = False
            innerbound = True
        elif innerbound and sensor_data['x_global'] <= landing_zone_start_position - safety_margin:
            near_x_limit = False
            outerbound = True
            innerbound = False
            
        if random_value == 0:
            if random.random() < 0.25 and sensor_data['x_global'] < x_lenght_arena - safety_margin * 3 :
                random_value = random.randint(20, 50)  # Duration for high-speed maneuver
                if random_value < 35: 
                    y_speed = -1.5  # Higher speed
                else:
                    y_speed = 1.5    
            else:
                random_value = random.randint(20, 1000)  # Longer duration for normal speed
                y_speed = random.uniform(0.03, 0.04)  # Normal random speed
        else:
            random_value -= 1  # Decrement after setting the speed
            
        vy = y_speed if not near_x_limit else -y_speed
    yaw = 0
    
    return vx, vy, yaw
        
def realign_to_x_axis(current_yaw, angle):
    
    target_yaw = angle  # Lacet cible pour l'alignement avec l'axe X
    yaw_correction = target_yaw - current_yaw  # Calcul de la correction nécessaire
    
    return yaw_correction

def turning_180(current_yaw):
    global drone_state 
    # Arrêter tout mouvement linéaire
    vx = 0
    vy = 0
    
    # Définir le yaw cible pour une rotation de 180 degrés
    target_yaw = -np.pi/2  # normaliser pour rester entre 0 et 2*pi
    
    # Calcul de la différence de yaw nécessaire
    yaw_difference = target_yaw - current_yaw
    
    # Appliquer une vitesse de yaw pour tourner sur place
    yaw_speed = 1  # Vitesse de rotation, ajustez selon les besoins pour un contrôle plus fin
    yaw = np.sign(yaw_difference) * min(abs(yaw_difference), yaw_speed)

    # Vérifier si le drone a atteint l'orientation cible
    if abs(yaw_difference) < 0.05:  # seuil pour considérer que le drone est correctement orienté
        yaw = 0  # arrêter la rotation
        drone_state = SHIFTING_RIGHT  # passer à l'état suivant
    else:
        drone_state = TURNING_180  # continuer la rotation

    return vx, vy, yaw

def realign_to_y_axis(current_yaw, angle):

    target_yaw = angle  # Lacet cible pour l'alignement avec l'axe y        
    yaw_correction = target_yaw - current_yaw  # Calcul de la correction nécessaire
    
    return yaw_correction

def turn_by_angle(current_yaw, target_angle, yaw_rate=0.005):

    if target_angle - current_yaw < yaw_rate:
        angle_reached = True
    else:
        angle_reached = False 
    
    return angle_reached

def shift():
    global drone_state, shift_start_time, vx, vy, yaw, shift_duration
    
    vx, vy, yaw = 0, 1, 0  # Start turning
    
    if shift_start_time is None:
        shift_start_time = time.time()   
    elif time.time() - shift_start_time > shift_duration:
        vx, vy, yaw = 0, 0, 0  # Stop turning
        drone_state = SWEEPING  # Move to the next state
        
    return  vx, vy, yaw

def check_boundaries(sensor_data, x_length, y_length, margin, vx, vy):
    if sensor_data['x_global'] >= x_length - margin:
        vx = 0
    if sensor_data['x_global'] <= 0 + margin:
        vx = 0
    if sensor_data['y_global'] >= y_length - margin:
        vy = 0
    if sensor_data['y_global'] <= 0 + margin:
        vy = 0   
    return vx, vy

def follow_outline(sensor_front, sensor_left, sensor_right, direction):
    global post_obstacle_counter
    
    x_speed = 0
    y_speed = 0
    
    if sensor_front < front_dist:
        # Obstacle détecté à l'avant : arrêt et déplacement latéral
        if direction == "right" :
            y_speed = 0.2
        else :
            y_speed = -0.2  
        
        post_obstacle_counter = post_obstacle_time  # Réinitialiser le compteur pour le front
    elif post_obstacle_counter > 0:
        # Décompter après la détection d'un obstacle à l'avant
        post_obstacle_counter -= 1
    else:
        if direction == "right" :
            # Aucun obstacle à l'avant et le compteur est à zéro, traiter le capteur gauche
            if sensor_left < critical_dist:
                # Obstacle détecté à gauche : ajuster la vitesse x pour suivre l'obstacle
                x_speed = 0.2  # Vitesse ajustée pour rester à une distance sûre
            else:
                # Aucun obstacle à gauche détecté ou besoin de continuer le mouvement normal
                x_speed = max_speed  # Vitesse normale en x quand il n'y a pas d'obstacles
            
            # Vérifie si l'obstacle de gauche nécessite un ajustement continu
            if sensor_left < critical_dist + 0.1:
                # Ajuster x_speed pour maintenir une distance constante avec l'obstacle
                x_speed = 0.2  # Réduire la vitesse pour un ajustement plus fin
        else :
            # Aucun obstacle à l'avant et le compteur est à zéro, traiter le capteur gauche
            if sensor_right < critical_dist:
                # Obstacle détecté à gauche : ajuster la vitesse x pour suivre l'obstacle
                x_speed = 0.2  # Vitesse ajustée pour rester à une distance sûre
            else:
                # Aucun obstacle à gauche détecté ou besoin de continuer le mouvement normal
                x_speed = max_speed  # Vitesse normale en x quand il n'y a pas d'obstacles
            
            # Vérifie si l'obstacle de gauche nécessite un ajustement continu
            if sensor_right < critical_dist + 0.1:
                # Ajuster x_speed pour maintenir une distance constante avec l'obstacle
                x_speed = 0.1  # Réduire la vitesse pour un ajustement plus fin
            
    drone_state = NORMAL
    return x_speed, y_speed
    
def turn_right(sensor_left) :
    global post_obstacle_counter
    
    x_speed = 0.1
    y_speed = 0
    
    if sensor_left < critical_dist:
        # Obstacle détecté à l'avant : arrêt et déplacement latéral
        y_speed = -0.4
        post_obstacle_counter = post_obstacle_time-5  # Réinitialiser le compteur pour le front
    elif post_obstacle_counter > 0:
        # Décompter après la détection d'un obstacle à l'avant
        post_obstacle_counter -= 1
    else:
    # Aucun obstacle à gauche détecté ou besoin de continuer le mouvement normal
        x_speed = max_speed*2  # Vitesse normale en x quand il n'y a pas d'obstacles
    
    drone_state = NORMAL
    return x_speed, y_speed

def turn_left(sensor_right) :
    global post_obstacle_counter
    
    x_speed = 0
    y_speed = 0
    
    if sensor_right < critical_dist:
        # Obstacle détecté à l'avant : arrêt et déplacement latéral
        y_speed = 0.4
        post_obstacle_counter = post_obstacle_time-5  # Réinitialiser le compteur pour le front
    elif post_obstacle_counter > 0:
        # Décompter après la détection d'un obstacle à l'avant
        post_obstacle_counter -= 1
    else:
    # Aucun obstacle à gauche détecté ou besoin de continuer le mouvement normal
        x_speed = max_speed *2 # Vitesse normale en x quand il n'y a pas d'obstacles
    
    drone_state = NORMAL
    return x_speed, y_speed
            
def stop() :
    global vx, vy, yaw
    vx = 0
    vy = 0  # No lateral movement if obstacles are on both sides
    yaw = 0
    
    return vx, vy, yaw

def unstuck(sensor_data, pos_x, pos_y):
        # Vérifie si le drone a bougé moins que le seuil de mouvement minimum
        if abs(pos_x - sensor_data['x_global']) < min_movement_threshold and abs(pos_y - sensor_data['y_global']) < min_movement_threshold:
            return True
        return False   
            
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = LoggingExample(uri)
    cf = le._cf

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)
    
    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    
    
    while le.is_connected:
        
        sensor_data['x_global'] = le.states['stateEstimate.x']
        sensor_data['y_global'] = le.states['stateEstimate.y']
        sensor_data['z_global'] = le.states['stateEstimate.z']
        sensor_data['range_down'] = le.states['range.up']
        sensor_data['range_front'] = le.states['range.front']
        sensor_data['range_right'] = le.states['range.right']
        sensor_data['range_left'] = le.states['range.left']
        sensor_data['range_back'] = le.states['range.back']
        sensor_data['yaw'] = le.states['stabilizer.yaw']
        time.sleep(0.01)

        [vx, vy, height_desired, yaw] = get_command(sensor_data)
        
        cf.commander.send_hover_setpoint(vx, vy, yaw, height_desired)
        time.sleep(0.1)
        
    