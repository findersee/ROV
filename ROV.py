import argparse
import os

import Pyro4
import RPi.GPIO as GPIO
from lowLevel import control

from edurov import WebMethod

def control_motors():
	with Pyro4.Proxy("PYRONAME:KeyManager") as keys:
	with Pyro4.Proxy("PYRONAME:ROVSyncer") as rov:
		while rov.run:
			if keys.state('K_UP'):
				print('Forward')
			elif keys.state('K_DOWN'):
				print('Backward')
			elif keys.state('K_RIGHT'):
				print('Right')
			elif keys.state('K_LEFT'):
				print('left')
			elif keys.state('K_a'):
				print('up')
			elif keys.state('K_z'):
				print('down')

def main(video_resolution='1024x768', fps=30, server_port=8000, debug=False):
    web_method = WebMethod(
        video_resolution=video_resolution,
        fps=fps,
        server_port=server_port,
        debug=debug,
        runtime_functions=control_motors,
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
