#!/usr/bin/env python3

import rospy
import csv
import os
import traceback
import numpy as np

from smach import State, StateMachine
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from drone.offboard import OffboardController

class Start(State):
    def __init__(self):
        State.__init__(self, outcomes = ['mission_acquired', 'failed'])
        self.count = 0

    def execute(self, _):
        try:
            rospy.loginfo('[FSM] Executing state START')
            self.count += 1

            attempts_count = rospy.get_param('~attempts_count')
            if self.count > attempts_count:
                rospy.delete_param('~target_waypoint')
                rospy.delete_param('~takeoff_altitude')

            while not (rospy.has_param('~target_waypoint') or rospy.has_param('~takeoff_altitude')):
                rospy.loginfo_once('[FSM] Waiting for target waypoint. Load target_waypoint and takeoff_altitude params.')

            rospy.loginfo('[FSM] Received target_waypoint and takeoff_height')
            return 'mission_acquired'
        except Exception as e:
            rospy.logerr(e)
            return 'failed'
        
class Takeoff(State):
    def __init__(self, drc):
        State.__init__(self, outcomes = ['takeoff_altitude_reached', 'failed'])
        self.drc = drc

    def execute(self, _):
        try:
            rospy.loginfo('[FSM] Executing state TAKEOFF')

            takeoff_altitude = rospy.get_param('~takeoff_altitude')

            self.drc.arm(timeout = 35.0)
            self.drc.reach_relative_goal([0, 0, takeoff_altitude], yaw = np.pi / 2, offset=0.15)
            
            return 'takeoff_altitude_reached'

        except Exception as e:
            traceback.print_exc()
            rospy.logerr(e)
            return 'failed'

class SearchAndLand(State):
    def __init__(self, drc, ena_pub):
        State.__init__(self, outcomes = ['landing_completed', 'failed'])
        self.drc = drc
        self.ena_pub = ena_pub
        self.target_detected = False
        self.pose_sub = None
        
    def _pose_callback(self, msg):
        """Callback pour détecter si une pose est disponible (plateforme détectée)"""
        # Si on reçoit une pose valide, la cible est détectée
        if not (msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0):
            self.target_detected = True
            rospy.loginfo('[FSM] Target detected during search!')
        
    def execute(self, _):
        try:
            rospy.loginfo('[FSM] Executing state SEARCH_AND_LAND')

            # Activer le contrôleur d'atterrissage
            msg_ena = Bool()
            msg_ena.data = True
            self.ena_pub.publish(msg_ena)
            rospy.loginfo('[FSM] Landing controller enabled for search')

            # S'abonner au topic de positionnement pour détecter la cible
            positioning_mode = rospy.get_param('/quadrotors_landing_vision/positioning', 'aruco')
            if positioning_mode == 'aruco':
                pose_topic = '/positioning/aruco'
            else:
                pose_topic = '/positioning/uwb_lqr_vanc'
                
            self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self._pose_callback)

            # Paramètres de recherche
            target_waypoint = rospy.get_param('~target_waypoint')
            
            # Aller vers la zone de recherche (centre de la zone)
            rospy.loginfo('[FSM] Moving to search area center')
            self.drc.reach_gps_goal(target_waypoint, yaw = np.pi / 2, offset = 1e-6)
            
            # Démarrer la recherche en tondeuse
            rospy.loginfo('[FSM] Starting lawnmower search pattern')
            search_center = self.drc.get_local_position()[:2]  # Position actuelle comme centre
            search_width = 40.0
            search_height = 40.0
            track_spacing = 5.0
            search_speed = 2.0
            search_altitude = 8.0
            
            # Générer les waypoints de la recherche en tondeuse
            waypoints = self._generate_lawnmower_pattern(
                search_center, search_width, search_height, track_spacing
            )
            
            rospy.loginfo(f'[FSM] Generated {len(waypoints)} search waypoints')
            
            # Exécuter la recherche
            for i, waypoint in enumerate(waypoints):
                rospy.loginfo(f'[FSM] Moving to search waypoint {i+1}/{len(waypoints)}')
                
                # Aller au waypoint de recherche
                target_pos = [waypoint[0], waypoint[1], search_altitude]
                
                # Se déplacer vers le waypoint en surveillant la détection
                start_pos = self.drc.get_local_position()
                
                while not self.target_detected and not rospy.is_shutdown():
                    current_pos = self.drc.get_local_position()
                    distance_to_target = np.linalg.norm(np.array(target_pos) - np.array(current_pos))
                    
                    # Si on est arrivé au waypoint, passer au suivant
                    if distance_to_target < 1.0:
                        break
                        
                    # Se déplacer vers le waypoint
                    self.drc.set_local_goal(target_pos, yaw=np.pi/2)
                    rospy.sleep(0.1)
                
                # Si la cible est détectée, commencer l'atterrissage
                if self.target_detected:
                    rospy.loginfo('[FSM] Target detected! Starting precision landing')
                    break
            
            # Si la cible n'a pas été trouvée après la recherche complète
            if not self.target_detected:
                rospy.logwarn('[FSM] Search completed but no target found. Continuing with landing controller...')
            
            # Maintenant, laisser le landing controller prendre le contrôle
            rospy.loginfo('[FSM] Switching to precision landing mode')
            
            # Attendre que le landing controller prenne le contrôle
            lpos = self.drc.get_local_position()
            while self.drc.get_next_vel_setpoint()[0] == 0.0 and \
                  self.drc.get_next_vel_setpoint()[1] == 0.0 and \
                  self.drc.get_next_vel_setpoint()[2] == 0.0:
                self.drc.set_local_goal(lpos, yaw = np.pi / 2)
                rospy.sleep(0.1)
            
            # Exécuter l'atterrissage de précision
            t = rospy.Time.now()
            while not rospy.is_shutdown():
                goal = self.drc.get_next_vel_setpoint()
                
                if not np.isnan(goal[2]):
                    if np.isinf(goal[2]):
                        rospy.logwarn('[FSM] Target lost during landing. Returning to search...')
                        # Reprendre la recherche depuis le waypoint suivant
                        self.target_detected = False
                        continue
                    
                    self.drc.set_velocity_goal(goal)
                else:
                    # Landing terminé
                    break

            self.drc.force_disarm()
            
            rospy.loginfo('[FSM] Landing completed')
            time = rospy.Time.now() - t
            rospy.loginfo('[FSM] Took {} seconds to land.'.format(time.to_sec()))

            # Calculer l'erreur d'atterrissage
            try:
                rospy.wait_for_service('/gazebo/get_model_state', 5)
            except rospy.ROSException:
                rospy.logerr('[FSM] Failed to connect to ModelState service')

            model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            error_resp = model_state('iris_bottom_fpv', 'husky')
            error_pos = np.array([error_resp.pose.position.x, error_resp.pose.position.y])

            error = np.linalg.norm(error_pos)
            rospy.loginfo('[FSM] Landing error: {}'.format(error))    

            # Désactiver le contrôleur d'atterrissage
            msg_disa = Bool()
            msg_disa.data = False
            self.ena_pub.publish(msg_disa)

            # Enregistrer les résultats
            file = rospy.get_param('~output')
            exists = os.path.exists(file)
            with open(file, 'a', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)

                if not exists:
                    writer.writerow(['error', 'time'])

                writer.writerow(['{}'.format(error), '{}'.format(time.to_sec())])

            # Nettoyer les abonnements
            if self.pose_sub:
                self.pose_sub.unregister()

            return 'landing_completed'
            
        except Exception as e:
            rospy.logerr(e)
            traceback.print_exc()
            return 'failed'       

    def _generate_lawnmower_pattern(self, center, width, height, spacing):
        """Génère un pattern en tondeuse pour la recherche"""
        waypoints = []
        
        # Points de départ de la zone de recherche
        start_x = center[0] - width/2
        start_y = center[1] - height/2
        
        # Nombre de passes
        num_passes = int(height / spacing)
        
        for i in range(num_passes):
            y = start_y + i * spacing
            
            if i % 2 == 0:  # Passe de gauche à droite
                waypoints.append([start_x, y])
                waypoints.append([start_x + width, y])
            else:  # Passe de droite à gauche
                waypoints.append([start_x + width, y])
                waypoints.append([start_x, y])
        
        return waypoints

def start():
    rospy.init_node('drone_fsm')
    rospy.loginfo('Initialized drone_fsm node')

    sm = StateMachine(outcomes = ['FAULT'])
    d = OffboardController()
    ena_pub = rospy.Publisher('/landing/enabled', Bool, queue_size = 10)

    with sm:
        StateMachine.add('START', Start(),
                          transitions = {'mission_acquired' : 'TAKEOFF',
                                         'failed' : 'FAULT'})
        StateMachine.add('TAKEOFF', Takeoff(d),
                          transitions = {'takeoff_altitude_reached' : 'SEARCH_AND_LAND',
                                         'failed' : 'FAULT'})        
        StateMachine.add('SEARCH_AND_LAND', SearchAndLand(d, ena_pub),
                          transitions = {'landing_completed' : 'START',
                                         'failed' : 'FAULT'})

    outcome = sm.execute()

    rospy.spin()

if __name__ == '__main__':
    start()
