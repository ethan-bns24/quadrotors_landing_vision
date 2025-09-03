#!/usr/bin/env python3

import rospy
import numpy as np
from math import log10
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool

class LandingController():

    def __init__(self):
        rospy.init_node('landing_controller', anonymous=True)
        rospy.loginfo('Initialized landing_controller node')

        self.dt = 1 / 150
        self.rate = rospy.Rate(1 / self.dt)

        self.relative_pos = [0, 0, 0]
        self.prev_error = [0, 0]
        self.integral = [0, 0]
        self.I_prev = [0, 0]

        self.lp_filter_estimate = [0, 0]
        self.lp_filter_prev_estimate = [0, 0]

        self.prev_timestamp = None
        self.timestamp = None
        self.target_lost = False
        self.land_ena = False
        self.time = 0
        self.search_mode = False
        self.target_ever_detected = False

        # Variables pour le mode recherche
        self.search_waypoints = []
        self.current_waypoint_idx = 0
        self.search_center = [0, 0]

        self._read_config()
        self._setup_subscribers()   
        self._setup_publishers()

    def _read_config(self):
        self.pos_topic = rospy.get_param('~pose_topic', 'uwb_lqr_vanc')
        self.a = rospy.get_param('~a', 20)
        self.b = rospy.get_param('~b', -17)
        self.Vz0 = rospy.get_param('~Vz0', 1.5)
        self.alpha = rospy.get_param('~alpha', 0.7)
        
        # Paramètres de recherche
        search_config = rospy.get_param('~search', {})
        self.search_enabled = search_config.get('enabled', True)
        self.search_center_local = search_config.get('center_local', [0.0, 0.0])
        self.search_width = search_config.get('width', 40.0)
        self.search_height = search_config.get('height', 40.0)
        self.search_track_spacing = search_config.get('track_spacing', 5.0)
        self.search_speed = search_config.get('speed', 2.0)
        self.search_altitude = search_config.get('z_search', 8.0)
        self.reacquire_timeout = search_config.get('reacquire_timeout', 2.0)

    def _setup_subscribers(self):
        rospy.loginfo('Subscribing to {}'.format(self.pos_topic))
        self.pose_sub = rospy.Subscriber(self.pos_topic, PoseStamped, self._positioning_callback)

        rospy.loginfo('Subscribing to /landing/enabled')
        self.land_ena_sub = rospy.Subscriber('/landing/enabled', Bool, self._land_ena_callback)

    def _setup_publishers(self):
        rospy.loginfo('Will publish to /landing/setpoint_vel')
        self.vel_pub = rospy.Publisher('/landing/setpoint_vel', TwistStamped, queue_size = 10)

    def _positioning_callback(self, msg):
        self.timestamp = msg.header.stamp

        # Si on reçoit une position non-nulle, la cible est détectée
        if not (msg.pose.position.x == 0 and msg.pose.position.y == 0 and msg.pose.position.z == 0):
            if not self.target_ever_detected:
                rospy.loginfo('[LANDING] Target detected for the first time!')
                self.target_ever_detected = True
                self.search_mode = False
            
            self.relative_pos[0] = msg.pose.position.x
            self.relative_pos[1] = msg.pose.position.y
            self.relative_pos[2] = msg.pose.position.z
        else:
            # Cible perdue
            if self.target_ever_detected:
                rospy.logwarn('[LANDING] Target lost!')

    def _land_ena_callback(self, msg):
        old_state = self.land_ena
        self.land_ena = msg.data
        
        if self.land_ena and not old_state:
            rospy.loginfo('[LANDING] Landing controller enabled')
            # Si la recherche est activée et qu'on n'a jamais détecté la cible, activer le mode recherche
            if self.search_enabled and not self.target_ever_detected:
                self.search_mode = True
                self._initialize_search()

    def _initialize_search(self):
        """Initialize the search pattern"""
        rospy.loginfo('[LANDING] Initializing search pattern')
        self.search_center = self.search_center_local.copy()
        self.search_waypoints = self._generate_lawnmower_pattern()
        self.current_waypoint_idx = 0
        rospy.loginfo(f'[LANDING] Generated {len(self.search_waypoints)} search waypoints')

    def _generate_lawnmower_pattern(self):
        """Generate lawnmower pattern waypoints"""
        waypoints = []
        
        start_x = self.search_center[0] - self.search_width/2
        start_y = self.search_center[1] - self.search_height/2
        
        num_passes = int(self.search_height / self.search_track_spacing)
        
        for i in range(num_passes):
            y = start_y + i * self.search_track_spacing
            
            if i % 2 == 0:  # Left to right
                waypoints.append([start_x, y])
                waypoints.append([start_x + self.search_width, y])
            else:  # Right to left
                waypoints.append([start_x + self.search_width, y])
                waypoints.append([start_x, y])
        
        return waypoints

    def _execute_search_pattern(self, current_pos):
        """Execute search pattern and return velocity command"""
        if self.current_waypoint_idx >= len(self.search_waypoints):
            # Search completed, hover and wait
            return [0, 0, 0]
        
        target_waypoint = self.search_waypoints[self.current_waypoint_idx]
        target_pos = [target_waypoint[0], target_waypoint[1], self.search_altitude]
        
        # Calculate distance to current waypoint
        distance = np.linalg.norm(np.array(target_pos[:2]) - np.array(current_pos[:2]))
        
        if distance < 1.0:  # Reached waypoint
            self.current_waypoint_idx += 1
            rospy.loginfo(f'[SEARCH] Reached waypoint {self.current_waypoint_idx}/{len(self.search_waypoints)}')
            
            if self.current_waypoint_idx >= len(self.search_waypoints):
                rospy.loginfo('[SEARCH] Search pattern completed')
                return [0, 0, 0]
        
        # Move towards waypoint
        direction = np.array(target_pos[:2]) - np.array(current_pos[:2])
        if np.linalg.norm(direction) > 0:
            direction = direction / np.linalg.norm(direction)
            
        # Maintain search altitude
        altitude_error = self.search_altitude - current_pos[2]
        
        return [
            direction[0] * self.search_speed,
            direction[1] * self.search_speed,
            altitude_error * 0.5  # Simple altitude controller
        ]

    def _send_vel(self, vel):
        msg = TwistStamped()

        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = vel[0]
        msg.twist.linear.y = vel[1]
        msg.twist.linear.z = vel[2]

        self.vel_pub.publish(msg)

    def start(self):
        while not rospy.is_shutdown():            
            if not self.land_ena:
                self.target_lost = False
                self._send_vel([0, 0, 0])
                self.rate.sleep()
                continue

            # Check if we should be in search mode
            if self.search_enabled and not self.target_ever_detected:
                if not self.search_mode:
                    self._initialize_search()
                    self.search_mode = True
                
                # Execute search pattern
                command = self._execute_search_pattern(self.current_drone_pos)
                self._send_vel(command)
                self.rate.sleep()
                continue

            # Normal precision landing mode
            if self.relative_pos == [0, 0, 0]:
                self.target_lost = False
                # If we had detected target before but lost it, try to reacquire
                if self.target_ever_detected and self.search_enabled:
                    # Simple reacquisition: small circular pattern
                    self._send_vel([1, 0, 0])  # Move in a pattern to reacquire
                else:
                    self._send_vel([0, 0, 0])
                self.rate.sleep()
                continue

            command = [0, 0, 0]
            error = np.array(self.relative_pos)

            stamp = self.timestamp
            if stamp == self.prev_timestamp:
                self.time += self.dt
                if self.time >= 1.:
                    self.target_lost = True
            else:
                self.time = 0.
                self.target_lost = False

            for i in range(2):
                self.lp_filter_estimate[i] = (self.alpha * self.lp_filter_prev_estimate[i]) + (1 - self.alpha) * \
                      (self.lp_filter_estimate[i] - self.lp_filter_prev_estimate[i]) / self.dt
                self.lp_filter_prev_estimate[i] = self.lp_filter_prev_estimate[i]

                P = np.exp(-0.199)*np.exp(-0.098*error[2]) * error[i]
                D = np.exp(-9.21) * np.exp(0.321 *error[2]) * self.lp_filter_estimate[i]
                
                self.integral[i] += ((self.prev_error[i] + error[i]) / 2) * self.dt
                if abs(error[0]) <= 0.5 and abs(error[1]) <= 0.5:
                    I = self.I_prev[i]
                else:
                    I = (np.exp(-1.878) * np.exp(-0.154 * error[2])) * self.integral[i]

                self.prev_error[i] = error[i]
                self.I_prev[i] = I
                command[i] = (P + I + D)          

            if abs(error[0]) <= 0.45 and abs(error[1]) <= 0.5:
                if abs(error[2]) <= 0.35:
                    command[2] = np.nan  # Signal landing completion
                else:
                    takeoff_alt = rospy.get_param('/drone_fsm/takeoff_altitude', 10)
                    x = max(1, abs(error[2] / takeoff_alt))
                    command[2] = -self.Vz0 * log10(1 + self.a * x + self.b * x ** 2)

            if self.target_lost:
                if self.time >= self.reacquire_timeout:
                    if self.search_enabled:
                        # Return to search mode
                        rospy.logwarn('[LANDING] Target lost, returning to search mode')
                        self.search_mode = True
                        self.target_ever_detected = False
                        self._initialize_search()
                        command = [0, 0, 5]  # Climb a bit before searching
                    else:
                        command[2] = np.inf  # Signal mission failure
                    self.time = 0
                    self.relative_pos = [0, 0, 0]
                elif abs(error[2]) < 7:
                    command[2] = 5
                    command[0] = 0
                    command[1] = 0

            self.prev_timestamp = stamp
            self._send_vel(command)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        LandingController().start()
    except rospy.ROSInterruptException:
        pass
