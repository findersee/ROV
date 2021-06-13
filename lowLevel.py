import time

import RPi.GPIO as GPIO

from adafruit_servoself.kit import Servoself.kit


class control(object):
	kit = Servoself.kit(channels=16)
	turnRate = 0.1

 
	def __init__(self,Left,Right,depthLeft,depthRight
              ,maxMove=1,minMove=-1
              ,maxDepth=1,minDepth=-1
              ,stopLevel=0,neutralLevel=0.2):
     
		self.left = self.kit.continuous_servo[Left].throttle
		self.right = self.kit.continuous_servo[Right].throttle
		self.lDepth = self.kit.continuous_servo[depthLeft].throttle
		self.rDepth = self.kit.continuous_servo[depthRight].throttle

		self.leftLevel = stopLevel
  		self.left = self.leftLevel
		self.rightLevel = stopLevel
		self.right = self.rightLevel
		self.rDepth = stopLevel
		self.lDepth = stopLevel
  
		self.maxMove = maxMove
		self.minMove = minMove
		self.maxDepth = maxDepth
		self.minDepth = minDepth
  
		self.stopLevel = stopLevel
		self.neutralLevel = neutralLevel
    
		#self.kit.continuous_servo[self.left].throttle = self.leftLevel
		#self.kit.continuous_servo[self.right].throttle = self.rightLevel
		#self.kit.continuous_servo[self.lDepth].throttle = self.depthLevel
		#self.kit.continuous_servo[self.rDepth].throttle = self.depthLevel
		
	def forward(self,power):
		self.leftLevel = (self.maxMove/power)
		self.rightLevel = (self.maxMove/power)
		self.right = self.rightLevel
		self.left = self.leftLevel
  		#self.kit.continuous_servo[self.left].throttle = self.leftLevel
		#self.kit.continuous_servo[self.right].throttle = self.rightLevel
	
 	def reverse(self,power):
		self.leftLevel = (self.minMove/power)
		self.rightLevel = (self.minMove/power)
		self.right = self.rightLevel
		self.left = self.leftLevel
		#self.kit.continuous_servo[self.left].throttle = self.leftLevel
		#self.kit.continuous_servo[self.right].throttle = self.rightLevel
  
	def right(self):
		
		if self.leftLevel + self.turnRate < self.maxMove:
			turnLeft = self.leftLevel + self.turnRate
		else:
			turnLeft = self.maxMove
   
		if self.rightLevel - self.turnRate > self.minMove:
			turnRight = self.rightLevel - self.turnRate
		else:
			turnRight = minMove
		self.left = turnLeft
		self.right = turnRight
  
	def left(self):
		
		if self.leftLevel - self.turnRate > self.minMove:
			turnLeft = self.leftLevel - self.turnRate
		else:
			turnLeft = self.minMove
   
		if self.rightLevel + self.turnRate < self.minMove:
			turnRight = self.rightLevel + self.turnRate
		else:
			turnRight = self.maxMove
		self.left = turnLeft
		self.right = turnRight

	def stop(self):
		self.leftLevel = stopLevel
		self.rightLevel = stopLevel
  
		self.right = self.rightLevel
		self.left = self.leftLevel
  
	def dive(self,power):
		self.rDepth = self.neutralLevel+(1/power)
		self.lDepth = self.neutralLevel+(1/power)
	
	def surface(self):
		self.rDepth = 0
		self.lDepth = 0
  
	def close(self):
		self.kit = 0
  

class sensors(object):
    
    def __init__(self):
        
        