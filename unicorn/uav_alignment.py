#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
from gpiozero import *
import time
import atexit
import socket
import sys
import threading

# create object for each PWM motor control hat 
PWM_FREQUENCY = 60 
hats = [Adafruit_MotorHAT(addr = 0x60, freq = PWM_FREQUENCY), Adafruit_MotorHAT(addr = 0x61, freq = PWM_FREQUENCY)] 

# auto-disable motors on program exit
def turnOffMotors():
    for mh in hats:
        mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
    return


def createServerSocket(host, port):
    # create server socket for receiving remote commands
    HOST = host     # Symbolic name meaning all available interfaces
    PORT = port     # Arbitrary non-privileged port
    s = None
    for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC,
                                socket.SOCK_STREAM, 0, socket.AI_PASSIVE):

        af, socktype, proto, canonname, sa = res
        try:
            s = socket.socket(af, socktype, proto)
        except OSError as msg:
            s = None
            continue
        try:
            s.bind(sa)
            s.listen(1)
        except OSError as msg:
            s.close()
            s = None
            continue
        break

    if s is None:
        print('could not open socket')
        sys.exit(1)

    conn, addr = s.accept()
    print('Connected by', addr)
	
    return conn


def runStepper(stepper, numSteps, direction, style, limitSwitch, debugPin):
    # GPIO Pin specified has a limitSwitch (button) attach to determine when the 
    # alignment rail hits the stop
    # beginning debug pulse sequence
    debugPin.on()
    debugPin.off()
    stepperOn = False
    while True:
        if numSteps <= 0 and stepperOn == False:
            stepper.startAutoRun(direction)
            stepperOn = True
        elif numSteps > 0:
            stepper.step(numSteps, direction, style)
        else:
            time.sleep(0.1)

        if direction == Adafruit_MotorHAT.FORWARD and limitSwitch[0].is_pressed:
            print("Limit reached, reversing...")
            direction = Adafruit_MotorHAT.BACKWARD
            stepper.stopAutoRun()
            time.sleep(1)
            stepper.startAutoRun(direction)
            continue
        elif direction == Adafruit_MotorHAT.BACKWARD and limitSwitch[1].is_pressed:
            print("Limit reached, stopping...")
            stepper.stopAutoRun()
            break
        if runStepper.halt == True:
            print("Halting...")
            stepper.stopAutoRun()
            break
    #end while
    
    # ending debug pulse sequence
    debugPin.off()
    debugPin.on()
    debugPin.off()

    return
	
def spawnStepperThread(myStepper, speed, numSteps, direction, style, limitSwitch, debugPin):
    myStepper.setSpeed(speed)
    t = threading.Thread(target=runStepper, args=(myStepper, numSteps, 
        direction, style, limitSwitch, debugPin))
    t.start()
    return t

# check for valid cmd 'a', 'h', 'r', or 'e' and then execute motor control 
# based on that cmd.  The function takes an array of 2 for the remaining args.
# In order to control stacked motor, this function must be called with 
# different sets of args for each.  Threads to control all motors will then be
# spawned.
STEPS_PER_REV = 200
SPEED = 60
STEPS = -1
def runMotors(cmd, mh, motors, limitSwitch, t, debugPin):
    steppers = [mh.getStepper(STEPS_PER_REV, motors[0]), mh.getStepper(STEPS_PER_REV, motors[1])]
    if cmd == "a":
        # spawn thread to run motor, STEPS_PER_REV steps/rev, motor port #, speed=SPEED, 
        # single step, forward direction, double coil
        if t[0] == None and t[1] == None:
            for i in range(len(steppers)):
                t[i] = spawnStepperThread(steppers[i], SPEED, STEPS, 
                    Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.DOUBLE, limitSwitch, debugPin)
        else:
            if t[0].is_alive() or t[1].is_alive():
                print("Command in progress")
            else:
                for i in range(len(steppers)):
                    t[i] = spawnStepperThread(steppers[i], SPEED, STEPS, 
                        Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.DOUBLE, limitSwitch, debugPin)

    elif cmd == "h":
        if t[0] != None and t[1] != None:
            if t[0].is_alive() == True or t[1].is_alive() == True:
                runStepper.halt = True
                while t[0].is_alive() == True or t[1].is_alive() == True:
                    # wait
                    time.sleep(1)
                print("Command halted!")	
    elif cmd == "r":
        # spawn thread to run motor, STEPS_PER_REV steps/rev, motor port #, speed=SPEED, 
        # single step, reverse direction, double coil
        if t[0] == None and t[1] == None:
            for i in range(len(steppers)):
                t[i] = spawnStepperThread(steppers[i], SPEED, STEPS, 
                    Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE, limitSwitch, debugPin)
        else:
            if t[0].is_alive() == True or t[1].is_alive() == True:
                print("Command in progress")
            else:
                for i in range(len(steppers)):
                    t[i] = spawnStepperThread(steppers[i], SPEED, STEPS, 
                        Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE, limitSwitch, debugPin)
    else:
        print("Invalid command!")
                
    return t

# Main --------------------------------------------------
atexit.register(turnOffMotors)
runStepper.halt = False

# create server socket for receiving remote commands
s = createServerSocket(None, 50000)

# Loop forever to receive commands.  When a command is received, validate it and then
# spawn a thread to perform the specified motor action.
motorPairs = [(1,2), (1,2), (1,2), (1,2)]
limitSwitch = [[Button(18, True, 0.001),Button(27, True, 0.001)],
               [Button(4, True, 0.001),Button(17, True, 0.001)],
               [Button(22, True, 0.001),Button(23, True, 0.001)],
               [Button(24, True, 0.001),Button(25, True, 0.001)]]
debugPin = [OutputDevice(5), OutputDevice(6), OutputDevice(12), OutputDevice(13)]
threads = [[None, None], [None, None], [None, None], [None, None]] 
while True:
    data = s.recv(1024)
    # print(data)
    # if no data returned then socket has been closed
    if not data: break

    # Call routines to run motor hat 1 
    threads[0] = runMotors(data, hats[0], motorPairs[0], limitSwitch[0], threads[0], debugPin[0])
    
    # Call routines to run motor hat 2 
    threads[1] = runMotors(data, hats[1], motorPairs[1], limitSwitch[1], threads[1], debugPin[1])

    # reset state vars before next iteration
    runStepper.halt = False
