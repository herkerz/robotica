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

        self.q_table = defaultdict(lambda: np.zeros(2))
        self.learning_rate = 0.1
        self.discount_factor = 0.95
        self.epsilon = 0.1
        self.previous_enemy_health = None

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
    
    def acercar_enemigo(self, myvalues, othervalues):
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
        
        if angle_difference >= 0.1:
            command.fire()
            self.previous_fire_angle = last_fire_angle
            print("Firing at angle:", last_fire_angle)
        
        return thrust, steering, bearing

    def get_state(self, myvalues, othervalues, bearing, distance_to_enemy, turretbearing, turretdecl, thrust, steering):
        state = (
            int(bearing // 36),
            int(distance_to_enemy // 100),
            int(turretbearing // 36),
            int(turretdecl // 10),
            int(thrust // 3.33),
            int(steering // 3),
            int(myvalues[td['x']] // 100),
            int(myvalues[td['z']] // 100),
            int(myvalues[td['bearing']] // 36),
            int(othervalues[td['x']] // 100),
            int(othervalues[td['z']] // 100)
        )
        return state

    def choose_action(self, state):
        if np.random.rand() < self.epsilon:
            return np.random.choice([0, 1])  # Explorar
        return np.argmax(self.q_table[state])  # Explotar

    def update_q_table(self, state, action, reward, next_state):
        best_next_action = np.argmax(self.q_table[next_state])
        td_target = reward + self.discount_factor * self.q_table[next_state][best_next_action]
        td_error = td_target - self.q_table[state][action]
        self.q_table[state][action] += self.learning_rate * td_error

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
                
                
                if distance_to_enemy > 300:
                    thrust, steering, bearing = self.acercar_enemigo(myvalues, othervalues)
                    

                if distance_to_enemy < 500:
                    thrust, steering, bearing = self.circular_enemigo(myvalues, othervalues, command)
                    
                    
                print(f"Time: {myvalues[td['timer']]} Health: {myvalues[td['health']]} Distance to enemy: {distance_to_enemy}")

                state = self.get_state(myvalues, othervalues, bearing, distance_to_enemy, turretbearing, 0, thrust, steering)
                action = self.choose_action(state)
                turretdecl = action
                # Determine reward
                current_health = othervalues[td['health']]
                
                reward = 0
                if command.command == 11: 
                    if (self.previous_enemy_health - current_health) > 0 if self.previous_enemy_health is not None else False:
                        reward = (self.previous_enemy_health - current_health) if self.previous_enemy_health is not None else 0
                    else:
                        reward = -10

                next_state = self.get_state(myvalues, othervalues, bearing, distance_to_enemy, turretbearing, 0,  thrust, steering)
                self.update_q_table(state, action, reward, next_state)

                self.previous_enemy_health = current_health  # Update the previous health

                self.recorder.recordvalues(myvalues, othervalues, 0, 0, 0, turretbearing)
                command.send_command(myvalues[td['timer']],self.tank,thrust,
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
