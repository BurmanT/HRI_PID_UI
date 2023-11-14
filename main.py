"""
   PID Control UI for HRI
"""
import sys
import uselect
import time
import servo
import json
import ssd1306
from machine import Pin, SoftI2C, PWM, ADC, Timer
import sensors

serialPoll = uselect.poll()
serialPoll.register(sys.stdin, uselect.POLLIN)

i2c = SoftI2C(scl = Pin(7), sda = Pin(6))
display = ssd1306.SSD1306_I2C(128, 64, i2c)

display.text("HELLO!", 40,35,1)
display.show()

sens=sensors.SENSORS()
# connect servo
motor = servo.Servo(Pin(2))
# angle of motor when beam is leveled
motorCenter = 87
motor.write_angle(motorCenter)
globalMotor = motorCenter

# To Do: have limits on these coefficients 
# disable pid if run button has been clicked 
kp = 0.03 #0.03
ki = 0.000001
kd = 10 #10

previousTime = time.ticks_ms()

error = 0
lastError = 0
inputVal = 0
cumError = 0
rateError = 0

# setpoint is value of sensor that you want it to read when ball is at center of beam 
setPoint = 850

# set the error so that motor does not jerk 
sensors = sens.readpoint()
inputVal = sensors[0]
dist = (1/(sensors[0])) * 1000000
error = setPoint - inputVal
lastError = error
globalSensor = dist

STATE = 0

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
    #print(str(currentTime) + ", " + str(error )+ ", " + str(output))
    return output


def runPID():
    sensors = sens.readpoint()
    inputVal = sensors[0] # distance sensor value
    # linearize the sensor reading 
    dist = (1/(sensors[0])) * 1000000
    out = computePID(dist)
    # control the motor based on PID value 
    # have to change the output here relative to the motor when leveled and if the error is neg or pos
    angle = motorCenter + out
    angle = min(max(angle, 0), 180)

    motor.write_angle(int(angle))
    
    dict_obj = {}
    dict_obj["m"] = int(angle)
    dict_obj["s"] = int(dist)

    string = json.dumps(dict_obj)
    sys.stdout.write(string+"\n")


def readSerial():
    """
    reads a single character over serial.

    :return: returns the character which was read, otherwise returns None
    """
    return(sys.stdin.read(1) if serialPoll.poll(0) else None)

 
def handleJson(string):
    """
        string format
        '{
            "p": 10,
            "i": 0.0001,
            "d": 0,
            "r": 0,
        }'

    """
    dictionary = {}
    json_obj = json.loads(string)
    if("r" in json_obj):
        if(json_obj["r"] == 1):
            runPID()
        else: 
            if("p" in json_obj):
                # update p value 
                global kp 
                kp = int(json_obj["p"])
            if("i" in json_obj):
                # update i value 
                global ki 
                ki = int(json_obj["i"])
            if("d" in json_obj):
                #update d value 
                global kd 
                kd = int(json_obj["d"])
            sensors = sens.readpoint()
            inputVal = sensors[0] # distance sensor value
            # linearize the sensor reading 
            dist = (1/(sensors[0])) * 1000000
    
            dict_obj = {}
            dict_obj["s"] = dist

            string = json.dumps(dict_obj)
            sys.stdout.write(string+"\n")

final_string = ''
append = False

starttime = time.time()
timer_begin = False

while True:
    #runPID()
    #if(STATE == 0):
    #    runPID()
    # continuously read commands over serial and handles them
    message = readSerial()

    # start of json dict
    if(message == '{'):
        append = True 

    # end of json dict
    elif(message == '}'):
        append = False
        final_string += message
        handleJson(final_string)
        final_string = ""
        
    if(append):
        if(type(message) is str):
            final_string += message
    
    if(message == None and timer_begin == False):
        starttime = time.time()
        timer_begin = True
    elif(not message is None):
        if(timer_begin):
            display.fill(0)
            display.text("Connected!", 25,35,1)
            display.show()
            timer_begin = False
    
    if(timer_begin == True and time.time()-starttime > 1):
        display.fill(0)
        display.text("Device", 30,15,1)
        display.text("Not", 30,35,1)
        display.text("Connected", 30,55,1)
        display.show()


