"""
   12/08
   PID Control UI for HRI with the Manual mode button 
       Resets PID when you go from Manual to PID
       Has motor limits for manual and PID
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

display.text("PID Mode", 30,35,1)
display.show()

# select switch
switch_select = Pin(9, Pin.IN)
sens=sensors.SENSORS()
# connect servo
motor = servo.Servo(Pin(2))
# angle of motor when beam is leveled
motorCenter = 81
motor.write_angle(motorCenter)

# limit for how high the motor can go
motorLimit = 35

# To Do: have limits on these coefficients 
# disable pid if run button has been clicked 
kp = 0.05 
ki = 0.00001
kd = 10

previousTime = time.ticks_ms()
# setpoint is value of sensor that you want it to read when ball is at center of beam 
setPoint = 750

error = 0
lastError = 0
inputVal = 0
cumError = 0
rateError = 0

# set the error so that motor does not jerk 
sensors = sens.readpoint()
inputVal = sensors[0]
dist = (1/(sensors[0])) * 1000000
error = setPoint - inputVal
lastError = error
globalSensor = dist

STATE = 0

switch_state_select = False
last_switch_state_select = False
switched_select = False
run_manual = False 

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
    global STATE
    if(STATE == 1):
        sensors = sens.readpoint()
        inputVal = sensors[0] # distance sensor value
        # linearize the sensor reading 
        dist = (1/(sensors[0])) * 1000000
        
        out = computePID(dist)
        # control the motor based on PID value 
        # have to change the output here relative to the motor when leveled and if the error is neg or pos
        angle = motorCenter + out
        angle = min(max(angle, 0), 180)
        
        if(angle > motorLimit):
            motor.write_angle(int(angle))
        
        dict_obj = {}
        dict_obj["m"] = out
        dict_obj["s"] = dist

        string = json.dumps(dict_obj)
        sys.stdout.write(string+"\n")

def runManual():
    sensors = sens.readpoint()
    mot = sensors[1]
    dist = (1/(sensors[0])) * 1000000
    #print(str(dist) + ", " + str(mot))
    # to prevent motor from breaking
    if(mot > motorLimit):
        motor.write_angle(mot)


def readSerial():
    """
    reads a single character over serial.

    :return: returns the character which was read, otherwise returns None
    """
    return(sys.stdin.read(1) if serialPoll.poll(0) else None)

def handleJson(string):
    
    #string format
    #{
    #    "p": 10,
    #    "i": 0.0001,
    #    "d": 0,
    #    "r": 0,
    #}
    
    dictionary = {}
    json_obj = json.loads(string)
    if("r" in json_obj):
        if(json_obj["r"] == 1):
            global STATE 
            STATE = 1
            if("p" in json_obj):
                # update p value 
                global kp 
                kp = float(json_obj["p"])
            if("i" in json_obj):
                # update i value 
                global ki 
                ki = float(json_obj["i"])
            if("d" in json_obj):
                #update d value 
                global kd 
                kd = float(json_obj["d"])
        else:
            global STATE 
            STATE = 0

# reset readings after Manual Mode to PID mode
def resetReadings():
    motor.write_angle(motorCenter)
    global previousTime
    global cumError
    global lastError
    global rateError
    global error
    previousTime = time.ticks_ms()

    error = 0
    lastError = 0
    inputVal = 0
    cumError = 0
    rateError = 0

    # set the error so that motor does not jerk 
    sensors = sens.readpoint()
    inputVal = sensors[0]
    dist = (1/(sensors[0])) * 1000000
    error = setPoint - inputVal
    lastError = error
    globalSensor = dist


def selectpressed():
    #declare all global variables, include all flags
    global run_manual
    if(run_manual == False):
        run_manual = True
    else:
        run_manual = False
        display.fill(0)
        display.text("PID Mode", 25,35,1)
        display.show()

def selectNotpressed():
    if(run_manual == True):
        display.fill(0)
        display.text("Press for PID Control", 25,35,1)
        display.show()

def check_switch(p):
    global switch_state_select
    global switched_select
    global last_switch_state_select

    switch_state_select = switch_select.value()
         
    if switch_state_select != last_switch_state_select:
        switched_select = True
        
    if switched_select:
        if switch_state_select == 0:
            selectpressed()
        switched_select = False
        #time.sleep(0.1)
    
    #if(switched_select == False):
    #    selectNotpressed()
        
    last_switch_state_select = switch_state_select

final_string = ''
append = False

#setting up Timers
tim = Timer(0)
tim.init(period=50, mode=Timer.PERIODIC, callback=check_switch)
last = False

while True:
    if(run_manual == False):
        # if you came from manual Mode, reset the PID readings and the angle 
        if(last == True):
            last = False
            resetReadings()
        # continuously read commands over serial and handles them
        message = readSerial()

        if(message is None):
            runPID()
        else:
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
    else:
        display.fill(0)
        display.text("MANUAL Mode",20,35,1)
        display.show()
        last = True
        runManual()




