import argparse
import os
import subprocess
import time

import Pyro4
import RPi.GPIO as GPIO
import lowLevel

from edurovi import WebMethod
from edurovi.utils import detect_pi, free_drive_space, cpu_temperature


def TrimResponse(not_used, path):
    """Will be called by the web server if it not able to process by itself"""
    if path.startswith('/Neutral_Trim'):
        #print(path.split('=',1)[1])
        #system.diveTrim(int(path.split('=',1)[1]))
        return "200"
    else:
        return None


def control_motors():
    prevBtnState = [0,0,0,0,0,0,0,0,0,0,0]
    with Pyro4.Proxy("PYRONAME:KeyManager") as keys:
        with Pyro4.Proxy("PYRONAME:ROVSyncer") as rov:
            keys.set_mode(key='l', mode='toggle')
            lightBtn = False
            with Pyro4.Proxy("PYRONAME:PadManager") as pad:
                system = lowLevel.control()
                axes = [0,0,0,0]
                while rov.run:
                    try:
                        axes[0] = pad.axes[0]
                        axes[1] = pad.axes[1]
                        axes[2] = pad.buttons[6]
                        axes[3] = pad.buttons[7]
                    except:
                        axes[0] = 0
                        axes[1] = 0
                        axes[2] = 0
                        axes[3] = 0

                    if keys.state('K_d'):
                        axes[0] = 0.25
                    elif keys.state('K_a'):
                        axes[0] = -0.25
                    elif keys.state('K_w'):
                        axes[1] = -0.3
                    elif keys.state('K_s'):
                        axes[1] = 0.25
                        
                    if keys.state('K_z'):
                        #print('up')
                        #system.dive(-40)
                        axes[2] = 0.4
                    elif keys.state('K_x'):
                        #print('down')
                        axes[3] = 0.65
                    elif keys.state('K_u'):
                        system.disArm()
                        
                    if keys.state('K_i'):
                        #print('Camera Down')
                        system.camera_down()
                    elif keys.state('K_c'):
                        system.camera_center()
                    elif keys.state('K_k'):
                        #print('Camera Up')
                        system.camera_up()
                    
                    
                    try:
                        if pad.buttons[1] and prevBtnState[1] != pad.buttons[1]:
                            lightBtn = not lightBtn
                        if pad.buttons[0] and prevBtnState[0] != pad.buttons[0]:
                            system.neutralLevel = system.divePower
                        if pad.buttons[2] and prevBtnState[2] != pad.buttons[2]:            
                            system.diveActive = not system.diveActive
                        prevBtnState = pad.buttons
                        if pad.buttons[4]:
                            system.camera_down()
                        if pad.buttons[5]:
                            system.camera_up()
                        if pad.buttons[8]:
                            system.camera_center()
                    except Exception as e:
                        print("<p> Error: %s<p>" % str(e)) 
                        

   

                    if keys.state(76) or lightBtn:
                        system.Light(True)
                    else:
                        system.Light(False)
                    
                    


                    system.motControl(axes)
                    axes = [0,0,0,0]
                system.close()


def sensors():
    sensors=lowLevel.sensors()
    with Pyro4.Proxy("PYRONAME:ROVSyncer") as rov:
        while rov.run:
            sens = sensors.readSensors()
            #print(sens)
            rov.sensor = {
                'pressureWater': sens['pressure'],
                'tempWater': sens['temperature'],
                'batteryVoltage' : sens['voltage'],
                'leak_front' : sens['leak'][0],
                'leak_rear' : sens['leak'][1],
            }

def system_monitor():
    with Pyro4.Proxy("PYRONAME:ROVSyncer") as rov:
        while rov.run:
            rov.sensor = {'free_space': free_drive_space(),
                          'cpu_temp': cpu_temperature()}
            time.sleep(10)

def main(video_resolution='1920x1080', fps=30, server_port=8000, debug=False):
    web_method = WebMethod(
        video_resolution=video_resolution,
        fps=fps,
        server_port=server_port,
        debug=debug,
        runtime_functions=[control_motors, sensors, system_monitor],
        custom_response=TrimResponse,
    index_file=os.path.join(os.path.dirname(__file__), 'index.html')
    )
    web_method.serve()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Start a streaming video server on raspberry pi')
    parser.add_argument(
        '-r',
        metavar='RESOLUTION',
        type=str,
        default='1024x768',
        help='''resolution, use format WIDTHxHEIGHT or an integer''')
    parser.add_argument(
        '-fps',
        metavar='FRAMERATE',
        type=int,
        default=30,
        help='framerate for the camera (default 30)')
    parser.add_argument(
        '-d', '--debug',
        action='store_true',
        help='set to print debug information')

    args = parser.parse_args()

    main(
        video_resolution=args.r,
        fps=args.fps,
        server_port=8000,
        debug=args.debug
    )
