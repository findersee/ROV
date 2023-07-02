import time
import glob
import os
import RPi.GPIO as GPIO

import board
import busio

from adafruit_servokit import ServoKit

from edurovi.utils import serial_connection

i2c = busio.I2C(board.SCL, board.SDA)
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
ads = ADS.ADS1115(i2c)
ads.gain = 2
front_leak = 27
rear_leak = 22

ser = serial_connection()


class cameraControl(object):

    kit = ServoKit(channels=16)
    
    def __init___(self,channel=0,maxRotate=145,minRotate=45,center=90):
        self.maxRotate = maxRotate
        self.minRotate = minRotate
        
        self.center = center
        self.camera = self.kit.servo[cameraServo]
        #print(self.camera)
        self.Angle = center
        self.camera.angle = self.Angle
 
    def set(self,level):
        self.Angle = self.Angle*0.95+0.05*level
        if self.Angle > self.maxRotate:
            self.Angle = self.maxRotate
        if self.Angle < self.minRotate:
            self.Angle = self.minRotate
            
        self.camera.angle = self.Angle
        
 
    def up(self):
        if self.Angle  < self.maxRotate:
            self.Angle = self.Angle + 1
        else:
            self.Angle = self.maxRotate
        #print(self.cameraAngle)
        self.camera.angle = self.Angle 
    
    def down(self):
        if self.Angle  > self.minRotate:
            self.Angle = self.Angle - 1
        else:
            self.Angle = self.minRotate
        #print(self.cameraAngle)
        self.camera.angle = self.Angle 
    
    def camera_center(self):
        self.Angle = 90
        self.camera.angle = self.Angle 
    
    def close(self):
        self.kit = 0
        #GPIO.cleanup()
        #self.pwm = 0



class control(object):
    
    leftLevel = 0
    rightLevel = 0

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(13,GPIO.OUT)

    
    def __init__(self
              ,maxMove=100,minMove=-100
              ,maxDepth=100,minDepth=-100
              ,stopLevel=0,neutralLevel=13):
        
        self.l_ch = Left        
        self.r_ch = Right
        self.ld_ch = depthLeft
        self.rd_ch = depthRight        
    
        #print(self.camera)    
            
        self.stopLevel = stopLevel

        self.leftLevel = self.stopLevel
        self.rightLevel = self.stopLevel

        self.maxMove = maxMove
        self.minMove = minMove
        self.maxDepth = maxDepth
        self.minDepth = minDepth
        self._diveActive = False
        self._neutralLevel = 0
        
        self.divePwr = 0
        
    def hatTransmit(mesg,serial_connection):
        msg = str(mesg).encode()
        serial_connection.write(msg)

    def disArm(self):
        if self._diveActive:
            self._diveActive = False
    
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

        if left_power < 0:
            leftMsg = "l"+str(abs(left_power))
        else:
            leftMsg = "L"+str(abs(left_power))
            
        if right_power < 0:
            rightMsg = "r"+str(abs(left_power))
        else:
            rightMsg = "R"+str(abs(left_power))
        #self.kit.continuous_servo[self.l_ch].throttle = self.leftLevel
        #self.kit.continuous_servo[self.r_ch].throttle = self.rightLevel
        
        
        
        if self._diveActive:
            if padVal[2] != 0:
                dive = padVal[2]

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
            if padVal[2] != 0:
                dive = padVal[2]
                #print(dive,self.divePwr)
                if abs(dive-self.divePwr) > 0.5:
                    if self.divePwr == 0:
                        dive = round(dive*0.5,3)
                    else:
                        dive = round(self.divePwr*0.5,3)   
            else:
                dive = 0
        self.divePwr = max(min(self.maxDepth,dive)),self.minDepth)
        
        if self.divePwr < 0:
            diveMsg = "-"+str(abs(self.divePwr))
        else:
            diveMsg = "+"+str(abs(self.divePwr))        
        #self.kit.continuous_servo[self.rd_ch].throttle = self.divePwr
        #self.kit.continuous_servo[self.ld_ch].throttle = self.divePwr  
        
        message = "#"+rightMsg+leftMsg+diveMsg
        self.hatTransmit(message,ser)
        
    def setLightOn(self):
        self.hatTransmit("#I100",ser)
    def setLightOff(self):
        self.hatTransmit("#I0",ser)

    def Light(self,lightBool_):
        if lightBool_:
            self.hatTransmit("#I100",ser)
        else:
            self.hatTransmit("#I0",ser)


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
        #GPIO.cleanup()
        self.pwm = 0

class sensors(object):

    GPIO.setup(rear_leak,GPIO.IN)
    GPIO.setup(front_leak,GPIO.IN)
    GPIO.setup(low_leak,GPIO.IN)
    GPIO.setup(high_leak,GPIO.IN)

    def __init__(self):
        self.GAIN = 2     
        base_dir = '/sys/bus/w1/devices/'
        device_folder = glob.glob(base_dir + '28*')[0]
        self.device_file = device_folder + '/w1_slave'
        os.system('modprobe w1-gpio')
        #os.system('modprobe w1-therm')
        
        self.InputVoltage = 0
        self.Pressure = 0
        self.MidRail = 0

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

    def parseHat(self,serial_connection):
        factor = 0.2666667
        if serial_connection.inWaiting():
            msg = serial_connection.readline().decode().rstrip()

            
            if len(msg) >= 32:
                #V1:X.xxxV2:X.xxxV3X.xxxV4:X.xxx
                self.InputVoltage = float(msg.split(':')[1].split("V")[0])
                self.Pressure = round(((float(msg.split(':')[2].split("V")[0])-factor)/factor)*100,3)
                reset_input_buffer()
            #else:
                #return None

    def readSensors(self):
    
        self.parseHat(ser)
    
        sensors = {
        'pressure': self.Pressure,
        'temperature': self.read_temp(),
        'voltage' : self.InputVoltage,
        'leak' : self.readLeak(),
        }        
        return sensors
        
            
    def readLeak(self):
        leak_f = GPIO.input(front_leak)
        leak_r = GPIO.input(rear_leak)
        leak_l = GPIO.input(low_leak)
        leak_h = GPIO.input(high_leak)
        
        
        return leak_f,leak_r,leak_l,leak_h
        
    def close(self):    
        GPIO.cleanup()