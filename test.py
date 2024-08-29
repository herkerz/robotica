'''
UDP Telemetry Receiving and Episode Recording

This script receives the information from the simulator (the telemetry)
picks only the data that corresponds to one of the tanks, and sends
a command to the simulator to control the tank.

This code is used to register many episodes from the game and to create a database.

Real time Data Science

'''
import socket
from struct import *
import datetime, time
from TelemetryDictionary import telemetrydirs as td
import sys

import math
import numpy as np

from Command import Command
from Command import Recorder

class Controller:
    def __init__(self, tankparam):
        # UDP Telemetry port on port 4500
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tankparam = int(tankparam)
        port = 4601
        if (tankparam == 1):
            port = 4601
        elif (tankparam == 2):
            port = 4602
        self.server_address = ('0.0.0.0', port)
        print ('Starting up on %s port %s' % self.server_address)

        self.sock.bind(self.server_address)
        self.sock.settimeout(5)

        self.length = 84
        self.unpackcode = 'Lififfffffffffffffff'

        self.recorder = Recorder()

        self.tank = tankparam

    def read(self):
        data, address = self.sock.recvfrom(self.length)

        #print(f"Fps: {fps.fps}")

        # Take care of the latency
        if len(data)>0 and len(data) == self.length:
            # is  a valid message struct
            new_values = unpack(self.unpackcode,data)
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
        
        
    
    def acercar_enemigo(self, myvalues, othervalues, std):
        vec2d = (float(myvalues[td['x']]), float(myvalues[td['z']]))
        ## enemy tank position
        vec2d_enemy = (float(othervalues[td['x']]), float(othervalues[td['z']]))

        # bearing to enemy
        bearing = math.degrees(math.atan2(vec2d_enemy[1]-vec2d[1], vec2d_enemy[0]-vec2d[0]))
        
        bearing = (bearing + 360) % 360
        bearing = (bearing + std) % 360
        
        # print("bearing to enemy: ", bearing)
        # print("my bearing: ", myvalues[td['bearing']])
        
        
        if bearing > 180:
            steering = bearing - myvalues[td['bearing']] - 180
        else:   
            steering = bearing - myvalues[td['bearing']] + 180
    
        steering = np.clip(steering, -15, 15)
        
        if abs(steering) >= 15:
            thrust = 3.0
            steering = steering
        else:
            thrust = 10.0
            steering = steering

        return thrust, steering
    
    def circular_enemigo(self, myvalues, othervalues):
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

        return thrust, steering
        
    def run(self):
        if (self.tank == 1):
            command = Command('127.0.0.1', 4501)
        elif (self.tank == 2):
            command = Command('127.0.0.1', 4502)

        shouldrun = True
        std = np.random.uniform(0, 360)
        while (shouldrun):
            #try:
            
                
                tank1values = self.read()
                if int(tank1values[td['number']]) != 1:
                    continue

                tank2values = self.read()
                if int(tank2values[td['number']]) != 2:
                    continue  


                if (self.tank == 1):
                    myvalues =  tank1values
                    othervalues = tank2values
                else:
                    myvalues = tank2values
                    othervalues = tank1values
                    
                turretbearing = self.lock_enemigo(myvalues, othervalues)
                
                #print(f"Tank1: {tank1values[td['number']]}, {tank1values[td['bearing']]}, Tank 2: {tank2values[td['number']]}, {tank1values[td['bearing']]}")              
                print (f"Time: {myvalues[td['timer']]} Health: {myvalues[td['health']]}")
                #self.recorder.recordvalues(myvalues,othervalues)




                distance_to_enemy = math.sqrt((myvalues[td['x']] - othervalues[td['x']])**2 + (myvalues[td['z']] - othervalues[td['z']])**2)
                print("distance to enemy: ", distance_to_enemy)
                
                if distance_to_enemy > 0:
                    thrust, steering = self.acercar_enemigo(myvalues, othervalues,std)
                    turretdecl = 0.0

                
                # if distance_to_enemy < 350:
                #     thrust, steering = self.circular_enemigo(myvalues, othervalues)
                #     turretdecl = 0.0
                    
                # if polardistance < 2700:
                #     thrust = 10.0
                #     steering = steering

                # If you want to FIRE the weapon no do this:
                # command.fire()
                #   This will set the variable command.command to 11 and will fire the weapon on the simulator.
                #   This will fire only once, because the alue will be reset the next time.
                #if (int(myvalues[td['timer']]) == 6000):
                #    # This allow RESETS of the simulation
                #    command.command = 13
                #    shouldrun = False

                command.send_command(myvalues[td['timer']],self.tank,thrust,
                                     steering,
                                     turretdecl,
                                     turretbearing)             
                
            #except socket.timeout:
            #    print("Episode Completed")
            #    break

if __name__ == '__main__':
    controller = Controller(sys.argv[1])
    controller.run()    

