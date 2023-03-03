
import pigpio
from time import sleep
import os
import sys
import time
import numpy as np
import smbus
from imusensor.MPU9250 import MPU9250
import RPi.GPIO as GPIO 

Pin1          = 12   # ���� ��
Pin2          = 13
Pin3          = 15
Pin4          = 16
SERVO_MAX_DUTY    = 12   # ������ �ִ�(180��) ��ġ�� �ֱ�
SERVO_MIN_DUTY    = 3    # ������ �ּ�(0��) ��ġ�� �ֱ�

GPIO.setmode(GPIO.BOARD)        # GPIO ����
GPIO.setup(Pin1, GPIO.OUT)
servo1 = GPIO.PWM(Pin1, 50) 
servo1.start(0)  
GPIO.setup(Pin2, GPIO.OUT)
servo2 = GPIO.PWM(Pin1, 50)  
servo2.start(0)
GPIO.setup(Pin3, GPIO.OUT)
servo3 = GPIO.PWM(Pin1, 50) 
servo3.start(0)
GPIO.setup(Pin4, GPIO.OUT)
servo4 = GPIO.PWM(Pin1, 50)  
servo4.start(0)

  
  
  




def Servo1(degree):
  
  if degree > 180:
    degree = 180

  
  duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)

  
  servo1.ChangeDutyCycle(duty)

def Servo2(degree):
  
  if degree > 180:
    degree = 180

  
  duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)

  
  servo2.ChangeDutyCycle(duty)  

def Servo3(degree):
  
  if degree > 180:
    degree = 180

  
  duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)

  
  servo3.ChangeDutyCycle(duty)

def Servo4(degree):
  
  if degree > 180:
    degree = 180

  
  duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)

  
  servo4.ChangeDutyCycle(duty)

if __name__ == "__main__":  
  # ���� 0���� ��ġ
  Servo1(0)
  sleep(1) # 1�� ���
  
  # ���� PWM ����
  servo1.stop()
  # GPIO ��� �ʱ�ȭ
  GPIO.cleanup()

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()




class param:
    pos0 = np.zeros((3,1))
    pos1 = np.zeros((3,1))
    pos2 = np.zeros((3,1))
    
    
    


imu.readSensor()
imu.computeOrientation()
param.pos1[0] = imu.AccelVals[0]
param.pos1[1] = imu.AccelVals[1]
param.pos1[2] = imu.AccelVals[2]

print ("Accel x: {0} ; Accel y : {1} ; Accel z : {2}".format(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]))   

position = True

while position == True:

    val = input()
    startmotor(val)
   
    def startmotor(val):
    
        if val == "start":
       

        
            try:
                while True:
                    imu.readSensor()
                    imu.computeOrientation()
                    time.sleep(0.1)
                
                    param.pos2[0] = imu.AccelVals[0]
                    param.pos2[1] = imu.AccelVals[1]
                    param.pos2[2] = imu.AccelVals[2]
                    print("pos2: ",param.pos2)
                    print ("Accel x: {0} ; Accel y : {1} ; Accel z : {2}".format(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]))
                
            
               
                    

               




            except KeyboardInterrupt:
