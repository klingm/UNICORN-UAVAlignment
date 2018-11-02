#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
from gpiozero import Button
import time
import atexit
import socket
import sys
import threading

# create object for PWM motor control (default I2C addr and freq)
mh = Adafruit_MotorHAT(addr = 0x60)

# auto-disable motors on program exit
def turnOffMotors():
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


def runStepper(stepper, numSteps, direction, style, limitSwitch):
    # GPIO Pin specified has a limitSwitch (button) attach to determine when the 
    # alignment rail hits the stop
    while True:
        if numSteps == -1:
            stepper.oneStep(direction, style)
        else:
            stepper.step(numSteps, direction, style);
        if direction == Adafruit_MotorHAT.FORWARD and limitSwitch[0].is_pressed:
            print("Limit reached, reversing...")
            direction = Adafruit_MotorHAT.BACKWARD
            time.sleep(1)
            continue
        elif direction == Adafruit_MotorHAT.BACKWARD and limitSwitch[1].is_pressed:
            print("Limit reached, stopping...")
            break
        if runStepper.halt == True:
            print("Halting...")
            break
    return
	
def spawnStepperThread(myStepper, speed, numSteps, direction, style, gpioNum):
    myStepper.setSpeed(speed)
    t = threading.Thread(target=runStepper, args=(myStepper, numSteps, 
        direction, style, gpioNum))
    t.start()
    return t

def runMotors(data, motors, limitSwitch, t):
    steppers = [mh.getStepper(100, motors[0]), mh.getStepper(100, motors[1])]
    if data == "a":
        # spawn thread to run motor, 100 steps/rev, motor port #, speed=500, 
        # single step, forward direction, double coil
        if t[0] == None and t[1] == None:
            for i in range(len(steppers)):
                t[i] = spawnStepperThread(steppers[i], 500, -1, 
                    Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.DOUBLE, limitSwitch)
        else:
            if t[0].is_alive() or t[1].is_alive():
                print("Command in progress")
            else:
                for i in range(len(steppers)):
                    t[i] = spawnStepperThread(steppers[i], 500, -1, 
                        Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.DOUBLE, limitSwitch)

    elif data == "h":
        if t[0] != None and t[1] != None:
            if t[0].is_alive() == True or t[1].is_alive() == True:
                runStepper.halt = True
                while t[0].is_alive() == True or t[1].is_alive() == True:
                    # wait
                    time.sleep(1)
                print("Command halted!")	
    elif data == "r":
        # spawn thread to run motor, 100 steps/rev, motor port #, speed=500, 
        # single step, reverse direction, double coil
        if t[0] == None and t[1] == None:
            for i in range(len(steppers)):
                t[i] = spawnStepperThread(steppers[i], 500, -1, 
                    Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE, limitSwitch)
        else:
            if t[0].is_alive() == True or t[1].is_alive() == True:
                print("Command in progress")
            else:
                for i in range(len(steppers)):
                    t[i] = spawnStepperThread(steppers[i], 500, -1, 
                        Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE, limitSwitch)
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
motorPairs = [(1,2), (3,4), (5,6), (7,8)]
limitSwitch = [[Button(4, True, 0.001),Button(17, True, 0.001)],
               [Button(18, True, 0.001),Button(27, True, 0.001)],
               [Button(22, True, 0.001),Button(23, True, 0.001)],
               [Button(24, True, 0.001),Button(25, True, 0.001)]]
threads = [[None, None], [None, None], [None, None], [None, None]] 
while True:
    data = s.recv(1024)
    # print(data)
    # if no data returned then socket has been closed
    if not data: break

    # Call routines to run motor 1
    threads[0] = runMotors(data, motorPairs[0], limitSwitch[0], threads[0])

    # reset state vars before next iteration
    runStepper.halt = False
