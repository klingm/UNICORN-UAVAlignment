#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
from gpiozero import *
import time
import atexit
import socket
import sys
import threading

# create object for each PWM motor control hat 
DEFAULT_PWM_FREQUENCY = 60
FAST_FREQ = 0
hats = [Adafruit_MotorHAT(addr = 0x60, freq = DEFAULT_PWM_FREQUENCY), 
        Adafruit_MotorHAT(addr = 0x61, freq = DEFAULT_PWM_FREQUENCY)] 

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
            stepper.startAutoRun(direction, FAST_FREQ)
            stepperOn = True
        elif numSteps > 0 and stepperOn == False:
            stepper.step(numSteps, direction, style)
        elif stepperOn == True:
            time.sleep(0.01)

        if direction == Adafruit_MotorHAT.FORWARD and limitSwitch[0].is_pressed:
            # stop the motors and then hold them in place
            print("Alignment position reached, holding...")
            stepper.stopAutoRun()

            # command the motors to hold 
            stepper.hold()
            break
        elif direction == Adafruit_MotorHAT.BACKWARD and limitSwitch[1].is_pressed:
            print("Stow position reached, stopping...")
            stepper.stopAutoRun()
            stepper.hold()
            break
        if runStepper.halt == True:
            print("Halting...")
            stepper.stopAutoRun()
            stepper.hold()
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

# check for valid cmd 'a1', 'a2', 'h', 'r', or 'e' and then execute motor control 
# based on that cmd.  The function takes an array of 2 for the remaining args.
# In order to control stacked motor, this function must be called with 
# different sets of args for each.  Threads to control all motors will then be
# spawned.
STEPS_PER_REV = 200
SPEED = 60
STEPS = 10
def runMotors(cmd, mh, motors, limitSwitch, t, debugPin):
    steppers = [mh.getStepper(STEPS_PER_REV, motors[0]), mh.getStepper(STEPS_PER_REV, motors[1])]
    if cmd == "a1" or cmd == "a2":
        steps = STEPS
        if cmd == "a1":
            steps = -1

        # spawn thread to run motor, STEPS_PER_REV steps/rev, motor port #, speed=SPEED, 
        # single step, forward direction, double coil
        if t[0] == None and t[1] == None:
            for i in range(len(steppers)):
                t[i] = spawnStepperThread(steppers[i], SPEED, steps, 
                    Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.MICROSTEP, limitSwitch, debugPin)
        else:
            if t[0].is_alive() or t[1].is_alive():
                print("Command in progress")
            else:
                for i in range(len(steppers)):
                    t[i] = spawnStepperThread(steppers[i], SPEED, steps, 
                        Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.MICROSTEP, limitSwitch, debugPin)

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
                t[i] = spawnStepperThread(steppers[i], SPEED, -1, 
                    Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.MICROSTEP, limitSwitch, debugPin)
        else:
            if t[0].is_alive() == True or t[1].is_alive() == True:
                print("Command in progress")
            else:
                for i in range(len(steppers)):
                    t[i] = spawnStepperThread(steppers[i], SPEED, -1, 
                        Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.MICROSTEP, limitSwitch, debugPin)
    else:
        print("Invalid command!" + cmd)
                
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
