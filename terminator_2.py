import socket
from struct import unpack
import datetime, time
import sys
import math
import numpy as np
import pickle
import random
from collections import defaultdict

from Command import Command
from Command import Recorder
from TelemetryDictionary import telemetrydirs as td


def default_q_value():
    return np.zeros(len(range(6)) * len(range(-5, 6)))  # Espacio de acciones combinadas


class Controller:
    def __init__(self, tankparam, load_q_table=False):
        # UDP Telemetry port on port 4500
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tankparam = int(tankparam)
        port = 4601 if tankparam == 1 else 4602
        self.server_address = ('0.0.0.0', port)
        print('Starting up on %s port %s' % self.server_address)

        self.sock.bind(self.server_address)
        self.sock.settimeout(5)

        self.length = 84
        self.unpackcode = 'Lififfffffffffffffff'

        self.recorder = Recorder()
        self.tank = tankparam
        self.previous_fire_angle = -3

        # Q-learning parameters
        self.q_table = defaultdict(default_q_value)
        self.learning_rate = 0.1
        self.discount_factor = 0.95
        self.epsilon = 0.1
        
        self.previous_enemy_health = None
        self.prev_x_enemy = 0
        self.prev_z_enemy = 0

        # Define the combined action space
        self.acciones = [(decl, bearing_corr) for decl in range(6) for bearing_corr in range(-5, 6)]

        if load_q_table:
            with open('q_table.pkl', 'rb') as f:
                self.q_table = pickle.load(f)
                print("Q-table loaded successfully.")

    def read(self):
        data, address = self.sock.recvfrom(self.length)
        if len(data) > 0 and len(data) == self.length:
            new_values = unpack(self.unpackcode, data)
            return new_values
        return None

    def lock_enemigo(self, myvalues, othervalues):
        vec2d = (float(myvalues[td['x']]), float(myvalues[td['z']]))
        ## enemy tank position
        vec2d_enemy = (float(othervalues[td['x']]), float(othervalues[td['z']]))

        # bearing to enemy
        bearing = math.degrees(math.atan2(vec2d_enemy[1]-vec2d[1], vec2d_enemy[0]-vec2d[0]))
        
        bearing = (bearing + 360) % 360
        bearing = (bearing + 90) % 360
        
        turretbearing = bearing - myvalues[td['bearing']] + 180
        
        return turretbearing
    
    def acercar_enemigo(self, myvalues, othervalues, command):
        vec2d = (float(myvalues[td['x']]), float(myvalues[td['z']]))
        ## enemy tank position
        vec2d_enemy = (float(othervalues[td['x']]), float(othervalues[td['z']]))

        # bearing to enemy
        bearing = math.degrees(math.atan2(vec2d_enemy[1]-vec2d[1], vec2d_enemy[0]-vec2d[0]))
        
        bearing = (bearing + 360) % 360
        bearing = (bearing + 90) % 360
        
        if bearing > 180:
            steering = bearing - myvalues[td['bearing']] - 180
        else:   
            steering = bearing - myvalues[td['bearing']] + 180
    
        steering = np.clip(steering, -15, 15)
        
        if abs(steering) >= 15:
            thrust = 3.0
        else:
            thrust = 10.0
        
        last_fire_angle = bearing
        angle_difference = abs(last_fire_angle - self.previous_fire_angle) 
        
        if angle_difference >= 1:
            command.fire()
            self.previous_fire_angle = last_fire_angle
            print("Firing at angle:", last_fire_angle)

        return thrust, steering, bearing
    
    def circular_enemigo(self, myvalues, othervalues, command):
        vec2d = (float(myvalues[td['x']]), float(myvalues[td['z']]))
        ## enemy tank position
        vec2d_enemy = (float(othervalues[td['x']]), float(othervalues[td['z']]))

        # bearing to enemy
        bearing = math.degrees(math.atan2(vec2d_enemy[1]-vec2d[1], vec2d_enemy[0]-vec2d[0]))
        
        bearing = (bearing + 360) % 360
        bearing = (bearing + 90) % 360
        
        circle_bearing = (bearing + 90) % 360
        
        steering = (circle_bearing - myvalues[td['bearing']]) % 360
        
        # Normalize steering to be within -180 to 180 degrees
        if steering > 180:
            steering -= 360
        
        if abs(steering) >= 15:
            thrust = 3.0
        else:
            thrust = 10.0

        print("bearing: ", bearing, " my bearing: ", myvalues[td['bearing']], '  steering: ', steering)
        print("steering: ", steering * 10)
        
        last_fire_angle = bearing
        angle_difference = abs(last_fire_angle - self.previous_fire_angle) 
        
        if angle_difference >= 1:
            command.fire()
            self.previous_fire_angle = last_fire_angle
            print("Firing at angle:", last_fire_angle)
        
        return thrust, steering, bearing

    def get_state(self, myvalues, othervalues, bearing, distance_to_enemy, turretbearing, turretdecl, thrust, steering, prev_x_enemy, prev_z_enemy):
        state = (
            int(bearing // 36),
            int((distance_to_enemy + 0.000000001) // 100),
            int(turretbearing // 36),
            int(turretdecl // 10),
            int(thrust // 3.33),
            int(steering // 3),
            int(myvalues[td['x']] // 100),
            int(myvalues[td['z']] // 100),
            int(myvalues[td['bearing']] // 36),
            int(othervalues[td['x']] // 100),
            int(othervalues[td['z']] // 100),
            int(prev_x_enemy - othervalues[td['x']] // 100),
            int(prev_z_enemy - othervalues[td['z']] // 100),
            int(othervalues[td['bearing']] // 36)
        )
        return state

    def choose_action(self, state):
        if np.random.rand() < self.epsilon:
            return random.choice(self.acciones)  # Explorar
        return max(self.acciones, key=lambda a: self.q_table[state][self.acciones.index(a)])  # Explotar

    def update_q_table(self, state, action, reward, next_state):
        best_next_action = max(self.acciones, key=lambda a: self.q_table[next_state][self.acciones.index(a)])
        td_target = reward + self.discount_factor * self.q_table[next_state][self.acciones.index(best_next_action)]
        td_error = td_target - self.q_table[state][self.acciones.index(action)]
        self.q_table[state][self.acciones.index(action)] += self.learning_rate * td_error

    def run(self):
        if self.tank == 1:
            command = Command('127.0.0.1', 4501)
        else:
            command = Command('127.0.0.1', 4502)

        shouldrun = True
        while shouldrun:
            try:
                tank1values = self.read()
                if int(tank1values[td['number']]) != 1:
                    continue

                tank2values = self.read()
                if int(tank2values[td['number']]) != 2:
                    continue  

                if self.tank == 1:
                    myvalues = tank1values
                    othervalues = tank2values
                else:
                    myvalues = tank2values
                    othervalues = tank1values

                turretbearing = self.lock_enemigo(myvalues, othervalues)
                distance_to_enemy = math.sqrt((myvalues[td['x']] - othervalues[td['x']]) ** 2 +
                                              (myvalues[td['z']] - othervalues[td['z']]) ** 2)
                
                thrust, steering, bearing = self.acercar_enemigo(myvalues, othervalues, command)

                print(f"Time: {myvalues[td['timer']]} Health: {myvalues[td['health']]} Distance to enemy: {distance_to_enemy}")

                state = self.get_state(myvalues, othervalues, bearing, distance_to_enemy, turretbearing, 0, thrust, steering, self.prev_x_enemy, self.prev_z_enemy)
                
                # Escoge la acción: combinación de turretdecl y corrección de bearing
                turretdecl, bearing_corr = self.choose_action(state)

                # Aplica la corrección al bearing
                turretbearing += bearing_corr

                # Determine reward
                current_health = othervalues[td['health']]
                
                reward = 0
                if command.command == 11: 
                    if (self.previous_enemy_health - current_health) > 0 if self.previous_enemy_health is not None else False:
                        reward = (self.previous_enemy_health - current_health) if self.previous_enemy_health is not None else 0
                    else:
                        reward = -50

                next_state = self.get_state(myvalues, othervalues, bearing, distance_to_enemy, turretbearing, turretdecl, thrust, steering, self.prev_x_enemy, self.prev_z_enemy)
                self.update_q_table(state, (turretdecl, bearing_corr), reward, next_state)

                self.previous_enemy_health = current_health  # Update the previous health
                self.prev_x_enemy = othervalues[td['x']]
                self.prev_z_enemy = othervalues[td['z']]
                
                self.recorder.recordvalues(myvalues, othervalues, turretdecl, bearing_corr, turretbearing, turretbearing)

                if (int(othervalues[td['health']]) < 50):
                    with open('q_table.pkl', 'wb') as f:
                        pickle.dump(self.q_table, f)
                        print("Q-table saved successfully.")
                    command.command = 13  
                    
                if (myvalues[td['timer']] > 4000):
                    with open('q_table.pkl', 'wb') as f:
                        pickle.dump(self.q_table, f)
                        print("Q-table saved successfully.")
                    command.command = 13  
                    
                    
                if (myvalues[td['power']] < 10):
                    with open('q_table.pkl', 'wb') as f:
                        pickle.dump(self.q_table, f)
                        print("Q-table saved successfully.")
                    command.command = 13  
                    
                command.send_command(myvalues[td['timer']], self.tank, thrust,
                                     steering,
                                     turretdecl,
                                     turretbearing)  
            except socket.timeout:
                print("Episode Completed")
                with open('q_table.pkl', 'wb') as f:
                    pickle.dump(self.q_table, f)
                    print("Q-table saved successfully.")
                break

if __name__ == '__main__':
    controller = Controller(sys.argv[1])
    controller.run()
