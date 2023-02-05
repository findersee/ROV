import time
import glob
import os
import RPi.GPIO as GPIO

import board
import busio
import RPi.GPIO as GPIO

from adafruit_servokit import ServoKit

i2c = busio.I2C(board.SCL, board.SDA)
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
ads = ADS.ADS1115(i2c)
ads.gain = 2
front_leak = 27
rear_leak = 22

class control(object):
    kit = ServoKit(channels=16)
    
    leftLevel = 0
    rightLevel = 0

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(13,GPIO.OUT)
    GPIO.setup(rear_leak,GPIO.IN)
    GPIO.setup(front_leak,GPIO.IN)
    
    def __init__(self,Left=0,Right=1,depthLeft=2,depthRight=3
              ,maxMove=1,minMove=-1
              ,maxDepth=1,minDepth=-1
              ,stopLevel=0,neutralLevel=0.13,turnRate=0.25,
              pulseMax=1100,pulseMin=1900,
              cameraServo=4,cameraMax=145,cameraMin=45):
        
        self.pwm = GPIO.PWM(13,10000)
        self.pwm.start(0)
        self.LightLevel= 0
        
        self.l_ch = Left        
        self.r_ch = Right
        self.ld_ch = depthLeft
        self.rd_ch = depthRight        
    
        self.camera = self.kit.servo[cameraServo]
        #print(self.camera)
        self.camera.angle = 90
        self.cameraAngle = 90
        self.turnRate = turnRate
        self.cameraMax = cameraMax
        self.cameraMin = cameraMin        

        self.kit.continuous_servo[self.r_ch].set_pulse_width_range(pulseMin,pulseMax)
        self.kit.continuous_servo[self.l_ch].set_pulse_width_range(pulseMin,pulseMax)
        self.kit.continuous_servo[self.ld_ch].set_pulse_width_range(pulseMin,pulseMax)
        self.kit.continuous_servo[self.rd_ch].set_pulse_width_range(pulseMin,pulseMax)
            
        self.stopLevel = stopLevel

        self.leftLevel = self.stopLevel
        self.kit.continuous_servo[self.l_ch].throttle = self.leftLevel
        self.rightLevel = self.stopLevel
        self.kit.continuous_servo[self.r_ch].throttle = self.rightLevel
        self.kit.continuous_servo[self.rd_ch].throttle = self.stopLevel
        self.kit.continuous_servo[self.ld_ch].throttle = self.stopLevel

        self.maxMove = maxMove
        self.minMove = minMove
        self.maxDepth = maxDepth
        self.minDepth = minDepth
        self._diveActive = False
        self._neutralLevel = 0
        
        self.divePwr = 0
        
        self.light = False

    def MoveStop(self):
        self.leftLevel = self.stopLevel
        self.rightLevel = self.stopLevel

        self.kit.continuous_servo[self.r_ch].throttle = self.rightLevel
        self.kit.continuous_servo[self.l_ch].throttle = self.leftLevel
        #print(self._diveActive)
        
    def DepthStop(self):
        if self._diveActive:
            #self.divePwr = self._neutralLevel
            #self.kit.continuous_servo[self.ld_ch].throttle = self._neutralLevel
            #self.kit.continuous_servo[self.rd_ch].throttle = self._neutralLevel
            divepower = self.divePwr
            if divepower > self._neutralLevel :
                while self._neutralLevel < divepower:
                    divepower = round(divepower - 0.1,2)
                    time.sleep(0.1)
                    if(divepower < self._neutralLevel):
                        divepower = self._neutralLevel
                        self.kit.continuous_servo[self.rd_ch].throttle = divepower
                        self.kit.continuous_servo[self.ld_ch].throttle = divepower
                        break
                    #print("divepower: " + str(divepower))
                    self.kit.continuous_servo[self.rd_ch].throttle = divepower
                    self.kit.continuous_servo[self.ld_ch].throttle = divepower
                #if self._neutralLevel+(power/100) > self.maxDepth:
                #    divepower = self.maxDepth
                #elif self._neutralLevel+(power/100) < self.minDepth:
                #    divepower = self.minDepth        
            if divepower < self._neutralLevel:
                divepower = self._neutralLevel
                #while self._neutralLevel > divepower:
                #    divepower = round(divepower + 0.1,2)
                    #print("divepower: " + str(divepower))
                 #   time.sleep(0.1)
                  #  if(divepower > self._neutralLevel):
                  #      divepower = self._neutralLevel
                   #     self.kit.continuous_servo[self.rd_ch].throttle = divepower
                    #    self.kit.continuous_servo[self.ld_ch].throttle = divepower
                     #   break
                self.kit.continuous_servo[self.rd_ch].throttle = divepower
                self.kit.continuous_servo[self.ld_ch].throttle = divepower
            self.divePwr = divepower
        
        
        
        else:
            self.divePwr = self.stopLevel
            self.kit.continuous_servo[self.rd_ch].throttle = self.stopLevel
            self.kit.continuous_servo[self.ld_ch].throttle = self.stopLevel

    def dive(self,powerCmd):
        if not self._diveActive:
            self._diveActive = True
        divepower = self._neutralLevel
        power = round(self._neutralLevel+(powerCmd/100),2)
        if power > self.maxDepth:
            power = self.maxDepth
        if power < self.minDepth:
            power = self.minDepth
        #print(self.divePwr , power)
        if power != self.divePwr:
            if power > 0:
                while divepower < power:
                    divepower = round(divepower + 0.05,2)
                    time.sleep(0.1)
                    if(divepower > self.maxDepth):
                        divepower = self.maxDepth
                        self.kit.continuous_servo[self.rd_ch].throttle = divepower
                        self.kit.continuous_servo[self.ld_ch].throttle = divepower 
                        break
                    #print("divepower: " + str(divepower))
                    self.kit.continuous_servo[self.rd_ch].throttle = divepower
                    self.kit.continuous_servo[self.ld_ch].throttle = divepower

                #if self._neutralLevel+(power/100) > self.maxDepth:
                #    divepower = self.maxDepth
                #elif self._neutralLevel+(power/100) < self.minDepth:
                #    divepower = self.minDepth        
            if power < 0:
                while divepower > power:
                    divepower = round(divepower - 0.05,2)
                    #print("divepower: " + str(divepower))
                    time.sleep(0.1)
                    if(divepower < self.minDepth):
                        divepower = self.minDepth
                        self.kit.continuous_servo[self.rd_ch].throttle = divepower
                        self.kit.continuous_servo[self.ld_ch].throttle = divepower
                        break
                    self.kit.continuous_servo[self.rd_ch].throttle = divepower
                    self.kit.continuous_servo[self.ld_ch].throttle = divepower
            self.divePwr = divepower
        #else:
        #    divepower = self._neutralLevel+(power/100)
            
        #print(divepower)
        #self.kit.continuous_servo[self.rd_ch].throttle = divepower
        #self.kit.continuous_servo[self.ld_ch].throttle = divepower

    def surface(self):
        self.kit.continuous_servo[self.rd_ch].throttle = -0.7
        self.kit.continuous_servo[self.ld_ch].throttle = -0.7

    def camera_up(self):
        if self.cameraAngle < self.cameraMax:
            self.cameraAngle = self.cameraAngle + 1
        else:
            self.cameraAngle = self.cameraMax
        #print(self.cameraAngle)
        self.camera.angle = self.cameraAngle
    
    def disArm(self):
        if self._diveActive:
            self._diveActive = False
    
    def camera_down(self):
        if self.cameraAngle > self.cameraMin:
            self.cameraAngle = self.cameraAngle - 1
        else:
            self.cameraAngle = self.cameraMin
        #print(self.cameraAngle)
        self.camera.angle = self.cameraAngle
    
    def camera_center(self):
        self.cameraAngle = 90
        self.camera.angle = self.cameraAngle
    
    def motControl(self, padVal):
        left_power = 0
        right_power = 0
        if (padVal[0] < -0.2 or padVal[0] > 0.2) or (padVal[1] < -0.2 or padVal[1] > 0.2):
            if padVal[0] < 0:
                padVal[0] = padVal[0]+0.05
            elif padVal[0] > 0:
                padVal[0] = padVal[0]-0.05       

            if padVal[1] < 0:
                padVal[1] = padVal[1]+0.05
            elif padVal[1] > 0:
                padVal[1] = padVal[1]-0.05 

            left_power = max(min(self.maxMove,round(padVal[1]+padVal[0],3)),self.minMove)
            right_power = max(min(self.maxMove,round(padVal[1]-padVal[0],3)),self.minMove)
            
        self.leftLevel = left_power
        self.rightLevel = right_power

        self.kit.continuous_servo[self.l_ch].throttle = self.leftLevel
        self.kit.continuous_servo[self.r_ch].throttle = self.rightLevel
        
        if self._diveActive:
            if padVal[3] != 0 or padVal[2] != 0:
                dive = padVal[3]-padVal[2]

                dive = dive+self._neutralLevel
                #print(dive,self.divePwr)
                if abs(dive-self.divePwr) > 0.5:
                    if self.divePwr == 0:
                        dive = round(dive*0.5,3)
                    else:
                        dive = round(self.divePwr*0.5,3)

            else:
                dive = self._neutralLevel 
                
                
        else:
            if padVal[3] != 0 or padVal[2] != 0:
                dive = padVal[3]-padVal[2]
                #print(dive,self.divePwr)
                if abs(dive-self.divePwr) > 0.5:
                    if self.divePwr == 0:
                        dive = round(dive*0.5,3)
                    else:
                        dive = round(self.divePwr*0.5,3)   
            else:
                dive = 0
        self.divePwr = max(min(self.maxDepth,dive)),self.minDepth)
        
        self.kit.continuous_servo[self.rd_ch].throttle = self.divePwr
        self.kit.continuous_servo[self.ld_ch].throttle = self.divePwr  
    
        
    def setLightOn(self):
        self.pwm.ChangeDutyCycle(100)
    def setLightOff(self):
        self.pwm.ChangeDutyCycle(0)

    def Light(self,lightBool_):
        if lightBool_:
            self.pwm.ChangeDutyCycle(100)
        else:
            self.pwm.ChangeDutyCycle(0)


    @property
    def neutralLevel(self):
        return self._neutralLevel

    @neutralLevel.setter
    def neutralLevel(self, nLevel):
        self._neutralLevel = nLevel

    @property
    def diveActive(self):
        return self._diveActive

    @diveActive.setter
    def diveActive(self, diveActive_):
        self._diveActive = diveActive_

    @property
    def divePower(self):
        return self.divePwr


    def diveTrim(self,trimValue):
        self._neutralLevel = trimValue/100
        
        print(self._neutralLevel)
        
        if self._diveActive:
            self.kit.continuous_servo[self.ld_ch].throttle = self._neutralLevel
            self.kit.continuous_servo[self.rd_ch].throttle = self._neutralLevel
        
    def close(self):
        self.kit.continuous_servo[self.l_ch].throttle = self.stopLevel
        self.kit.continuous_servo[self.r_ch].throttle = self.stopLevel
        self.kit.continuous_servo[self.rd_ch].throttle = self.stopLevel
        self.kit.continuous_servo[self.ld_ch].throttle = self.stopLevel
        self.kit = 0
        GPIO.cleanup()
        self.pwm = 0

class sensors(object):


    def __init__(self):
        self.GAIN = 2     
        base_dir = '/sys/bus/w1/devices/'
        device_folder = glob.glob(base_dir + '28*')[0]
        self.device_file = device_folder + '/w1_slave'
        os.system('modprobe w1-gpio')
        #os.system('modprobe w1-therm')

    def read_temp_raw(self):
        f = open(self.device_file, 'r')
        lines = f.readlines()
        f.close()
        return lines


    def read_temp(self):
        lines = self.read_temp_raw()
        try:
            while lines[0].strip()[-3:] != 'YES':
                time.sleep(0.2)
                lines = read_temp_raw()
            equals_pos = lines[1].find('t=')
        except:
            equals_pos = -1
        if equals_pos != -1:
            temp_string = lines[1][equals_pos+2:]
            temp = float(temp_string) / 1000.0
            return temp

    def read_volts(self):
        voltCh = AnalogIn(ads,ADS.P1)
        return voltCh.voltage*(258/18)

    def read_pressure(self):
        factor = 0.2666667
        presCh = AnalogIn(ads,ADS.P0)
        pressure = round(((presCh.voltage-factor)/factor)*100,3)
        return pressure

    def readSensors(self):
        sensors = {
        'pressure': self.read_pressure(),
        'temperature': self.read_temp(),
        'voltage' : self.read_volts(),
        'leak' : self.readLeak(),
        }        
        return sensors
        
            
    def readLeak(self):
        leak_f = GPIO.input(front_leak)
        leak_r = GPIO.input(rear_leak)
        return leak_f,leak_r
