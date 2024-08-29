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
    
    def run(self):
        if (self.tank == 1):
            command = Command('127.0.0.1', 4501)
        elif (self.tank == 2):
            command = Command('127.0.0.1', 4502)

        shouldrun = True
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

                #print(f"Tank1: {tank1values[td['number']]}, {tank1values[td['bearing']]}, Tank 2: {tank2values[td['number']]}, {tank1values[td['bearing']]}")              
                print (f"Time: {myvalues[td['timer']]} Health: {myvalues[td['health']]}")
                self.recorder.recordvalues(myvalues,othervalues)


                vec2d = (float(myvalues[td['x']]), float(myvalues[td['z']]))

                polardistance = math.sqrt( vec2d[0] ** 2 + vec2d[1] ** 2)

                print(polardistance)

                thrust = 0.0
                turretbearing = 0.0
                turretdecl = 0.0
                steering=0.0

                if polardistance < 1700:
                    thrust = 10.0
                    steering = 0

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

