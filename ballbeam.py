from machine import Pin, SoftI2C, PWM, ADC
from files import *
import time
from machine import Timer
import servo
import icons
import os
import sys
import ubinascii
import machine
import uselect
import sensors
sens=sensors.SENSORS()

#servo
s = servo.Servo(Pin(2))
serialPoll = uselect.poll()
serialPoll.register(sys.stdin, uselect.POLLIN)
i2c = SoftI2C(scl = Pin(7), sda = Pin(6))
display = icons.SSD1306_SMART(128, 64, i2c)
display.text("HELLO!", 40,35,1)
display.show()

## PID constants 
#kp = 0.005 # or 0.5
#ki = 0.000004
#ki=0
kp = 0.03
ki = 0.000001
kd = 10

previousTime = time.ticks_ms()

error = 0
lastError = 0
inputVal = 0
cumError = 0
rateError = 0

# setpoint is value of sensor that you want it to read when ball is at center of beam 
setPoint = 850
# angle of motor when beam is leveled
motorCenter = 82
s.write_angle(motorCenter)

def computePID(inputVal):
    global previousTime
    global cumError
    global lastError
    global rateError
    global error
    # get current time 
    currentTime = time.ticks_ms()
    # compute elapsed time from previous computation 
    elapsedTime = currentTime - previousTime
    # determine error 
    error = setPoint - inputVal
    # compute integral 
    cumError += error * elapsedTime
    # compute derivative 
    rateError = (error -  lastError)/elapsedTime
    output = kp * error + ki * cumError + kd * rateError
    lastError = error
    previousTime = currentTime
    #print("-----------------------------------------")
    #print("Error is: ")
    print(str(currentTime) + ", " + str(error )+ ", " + str(output))
    
    return output

# set the error so that motor does not jerk 
sensors = sens.readpoint()
inputVal = sensors[0]
dist = (1/(sensors[0])) * 1000000
error = setPoint - inputVal
lastError = error

while True:
    sensors = sens.readpoint()
    inputVal = sensors[0] # distance sensor value
    # linearize the sensor reading 
    dist = (1/(sensors[0])) * 1000000
    out = computePID(dist)
    # control the motor based on PID value 
    # have to change the output here relative to the motor when leveled and if the error is neg or pos
    angle = motorCenter + out
    angle = min(max(angle, 0), 180)

    #print("Angle to motor:")
    #print(angle)
    s.write_angle(int(angle))
    
"""
while True:
    sensors = sens.readpoint()
    motor = sensors[1]
    dist = (1/(sensors[0])) * 1000000
    print(str(dist) + ", " + str(motor))
    
    s.write_angle(motor)

"""  
