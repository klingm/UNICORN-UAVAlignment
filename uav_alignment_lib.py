#!/usr/bin/env python
try:
    from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
    from gpiozero import *
    import time
    import atexit
    import sys
    import threading
    import socket
    
except ImportError as Ie:
    print 'Import error ', Ie
    
def alignButtonCallback(_btn, _stepper1 = None, _stepper2 = None, _lock = None, _halt = False):
    print("alignButtonCallbacki called")
    if _halt == True:
        _stepper1.stopAutoRun()
        _stepper1.hold()
        _stepper2.stopAutoRun()
        _stepper2.hold()
        print("     Button state change detected, stopping motors...")
    else:
        print("     Button state change detected...")
    
    print("alignButtonCallback done")
    _lock.acquire(False)
    _lock.release()

def stowButtonCallback(_btn, _stepper1 = None, _stepper2 = None, _lock = None, _halt = False):
    print("stowButtonCallback called")
    if _halt == True:
        _stepper1.stopAutoRun()
        _stepper1.hold()
        _stepper2.stopAutoRun()
        _stepper2.hold()
        print("     Button state change detected, stopping motors...")
    else:
            print("     Button state change detected...")
    
    print("stowButtonCallback done")
    _lock.acquire(False)
    _lock.release()

class Unicorn_Docker:
    def __init__(self):
        try:
            # create object for each PWM motor control hat 
            self.DEFAULT_PWM_FREQUENCY = 47 
            self.FAST_FREQ = 0
            self.hats=[
                Adafruit_MotorHAT(addr = 0x60, freq = self.DEFAULT_PWM_FREQUENCY),
                Adafruit_MotorHAT(addr = 0x61, freq = self.DEFAULT_PWM_FREQUENCY),
                Adafruit_MotorHAT(addr = 0x62, freq = self.DEFAULT_PWM_FREQUENCY),
                Adafruit_MotorHAT(addr = 0x63, freq = self.DEFAULT_PWM_FREQUENCY)]
            self.STEPS_PER_REV = 200
            self.SPEED = 60
            self.STEPS = 10

            atexit.register(self._exit)
            self.halt = False

            self.s = None

            # spawn a thread to perform the specified motor action.
            self.motorPairs = [(1,2)]
            self.DEBOUNCE = 0.05
            self.limitSwitch=[
                        [Button(18, True, None, self.DEBOUNCE),Button(27, True, None, self.DEBOUNCE)],
                        [Button(4, True, None, self.DEBOUNCE), Button(17, True, None, self.DEBOUNCE)],
                        [Button(22, True, None, self.DEBOUNCE),Button(23, True, None, self.DEBOUNCE)],
                        [Button(24, True, None, self.DEBOUNCE),Button(25, True, None, self.DEBOUNCE)]]
            self.debugPin = [OutputDevice(5), OutputDevice(6), OutputDevice(12), OutputDevice(13)]
            self.threads = [[None, None], [None, None], [None, None], [None, None]] 

        except Exception as eInit:
            print 'Exception in init ', eInit
            #sys.exit()
    
    def do_action(self, data, join=True):
        # reset state vars before next iteration
        self.halt = False
        
        if data == "a1" or data == "r":
            # Call routines to run motor hat 1, 2, 3, 4
            for i in range(4):
                self.threads[i] = self.runMotors(data, 
                                            self.hats[i], 
                                            self.motorPairs[0], 
                                            self.limitSwitch[i], 
                                            self.threads[i], 
                                            self.debugPin[i])
            
            # block for each thread to exit
            if join == True and data != 'rh':
                print 'Waiting for all threads to exit...'
                for i in range(4):
                    print "Waiting on thread ", i
                    if self.threads[i][0].is_alive():
                        self.threads[i][0].join()
                    if self.threads[i][1].is_alive():
                        self.threads[i][1].join()
        
        elif data == "rh":
            # command motors to turn off (release hold)
            print("Turning off all motors, releasing hold...")
            self.turnOffMotors()
        elif data == "h":
            # halt the current action
            self.halt = True

    def run(self):
        # create server socket for receiving commands
        self.createServerSocket(None, 50000)
        
        while True:
            data = self.s.recv(1024)
            if not data: break

            self.do_action(data, False)

        self.s.close()

    def createServerSocket(self, host, port):
        # create server socket for receiving remote commands
        HOST = host     # Symbolic name meaning all available interfaces
        PORT = port     # Arbitrary non-privileged port
        s1 = None
        for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC,
                                socket.SOCK_STREAM, 0, socket.AI_PASSIVE):

            af, socktype, proto, canonname, sa = res
            try:
                s1 = socket.socket(af, socktype, proto)
            except OSError as msg:
                s1 = None
                continue
            try:
                s1.bind(sa)
                s1.listen(1)
            except OSError as msg:
                s1.close()
                s1 = None
                continue
            break

        if s1 is None:
            print('could not open socket')
            sys.exit(1)

        self.s, addr = s1.accept()
        print('Connected by', addr)
	
    # check for valid cmd 'a1', 'a2', 's', 'r', 'rh' and then execute motor control 
    # based on that cmd.  The function takes an array of 2 for the remaining args.
    # In order to control stacked motor, this function must be called with 
    # different sets of args for each.  Threads to control all motors will then be
    # spawned.

    def runMotors(self, cmd, mh, motors, limitSwitch, t, debugPin):
        if cmd != 'a1' and cmd != 'r':
            # invalid command!
            print("Invalid command passed to runMotors: " + cmd)
            return t

        steppers = [mh.getStepper(self.STEPS_PER_REV, motors[0]), 
                    mh.getStepper(self.STEPS_PER_REV, motors[1])]

        # set up limitSwitch callbacks
        stopMotors1 = False
        stopMotors2 = False
        lock1 = threading.Lock()
        lock2 = threading.Lock()
        lambda1 = lambda x: alignButtonCallback(limitSwitch[0], steppers[0], steppers[1], lock1, stopMotors1)
        lambda2 = lambda y: stowButtonCallback(limitSwitch[1], steppers[0], steppers[1], lock2, stopMotors2)
        limitSwitch[0].when_pressed = lambda1
        limitSwitch[1].when_pressed = lambda2

        # Handle command for aligning - move forward
        #if cmd == "a1" or cmd == "a2":
        if cmd == "a1":
            # set callback for limit switches
            stopMotors1= True
            limitSwitch[0].when_pressed = lambda1
            lock1.acquire()
            
            # run steppers
            steps = self.STEPS
            if cmd == "a1":
                steps = -1

            # spawn thread to run motor, STEPS_PER_REV steps/rev, motor port #, speed=SPEED, 
            # single step, forward direction, double coil
            if t[0] == None and t[1] == None:
                for i in range(len(steppers)):
                    t[i] = self.spawnStepperThread(steppers[i], 
                            self.SPEED, 
                            steps, 
                            Adafruit_MotorHAT.FORWARD, 
                            Adafruit_MotorHAT.MICROSTEP, 
                            limitSwitch, 
                            debugPin, 
                            lock1, 
                            lock2)
            else:
                if t[0].is_alive() or t[1].is_alive():
                    print("Command in progress")
                else:
                    for i in range(len(steppers)):
                        t[i] = self.spawnStepperThread(steppers[i], 
                                self.SPEED, 
                                steps, 
                                Adafruit_MotorHAT.FORWARD, 
                                Adafruit_MotorHAT.MICROSTEP, 
                                limitSwitch, 
                                debugPin, 
                                lock1, 
                                lock2)
        # Handle command for stow - move backward
        elif cmd == "r":
            # set callback for limit switches
            stopMotors2 = True
            limitSwitch[1].when_pressed = lambda2
            lock2.acquire()

            # reset to stowed position
            if t[0] == None and t[1] == None:
                for i in range(len(steppers)):
                    t[i] = self.spawnStepperThread(steppers[i], 
                            self.SPEED, 
                            -1, 
                            Adafruit_MotorHAT.BACKWARD, 
                            Adafruit_MotorHAT.MICROSTEP, 
                            limitSwitch, 
                            debugPin, 
                            lock1, 
                            lock2)
            else:
                if t[0].is_alive() == True or t[1].is_alive() == True:
                    print("Command in progress")
                else:
                    for i in range(len(steppers)):
                        t[i] = self.spawnStepperThread(steppers[i], 
                                self.SPEED, 
                                -1, 
                                Adafruit_MotorHAT.BACKWARD, 
                                Adafruit_MotorHAT.MICROSTEP, 
                                limitSwitch, 
                                debugPin, 
                                lock1, 
                                lock2)
        else:
            print("Invalid command!" + cmd)
                    
        return t
    
    # auto-disable motors on program exit
    def turnOffMotors(self):
        for mh in self.hats:
            mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
            mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
            mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
            mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
        return

    def _exit(self):
        #turnOffMotors()
        return

    def runStepper(self, stepper, numSteps, direction, style, limitSwitch, debugPin, lock1, lock2):
        # GPIO Pin specified has a limitSwitch (button) attach to determine when the 
        # alignment rail hits the stop
        # beginning debug pulse sequence
        debugPin.on()
        debugPin.off()
        stepperOn = False
        dontWait = False
        while True:
            if numSteps <= 0 and stepperOn == False and self.halt == False:
                if direction == Adafruit_MotorHAT.FORWARD and limitSwitch[0].is_pressed:
                    lock1.release()
                    dontWait = True
                    break
                if direction == Adafruit_MotorHAT.BACKWARD and limitSwitch[1].is_pressed:
                    lock2.release()
                    dontWait = True
                    break
                stepper.startAutoRun(direction, self.FAST_FREQ)
                stepperOn = True
                
            elif numSteps > 0 and stepperOn == False:
                stepper.step(numSteps, direction, style)
            elif stepperOn == True:
                time.sleep(1)
                if direction == Adafruit_MotorHAT.FORWARD and limitSwitch[0].is_pressed:
                    print("Alignment position reached, halting...")
                    break 
                elif direction == Adafruit_MotorHAT.BACKWARD and limitSwitch[1].is_pressed:
                    print("Stow position reached, halting...")
                    break
                else:
                    stepper.stopAutoRun()
                    stepper.hold()
                    time.sleep(1)
                    stepperOn = False

            if self.halt == True:
                print("Halting...")
                stepper.stopAutoRun()
                if direction == Adafruit_MotorHAT.FORWARD:
                    # ensure the mutex is released
                    lock1.acquire(False)
                    lock1.release()

                if direction == Adafruit_MotorHAT.BACKWARD:
                    # ensure the mutex is released
                    lock2.acquire(False)
                    lock2.release()

                break
        #end while

        # Wait for callbacks to exit before returning
        if not dontWait:
            # block on mutexes, if they were previously acquired.  When unblocked immediately release.
            lock1.acquire()
            lock1.release()
            lock2.acquire()
            lock2.release()
        
        print("runStepper exiting...")

        # ending debug pulse sequence
        debugPin.off()
        debugPin.on()
        debugPin.off()

        return
    
    def spawnStepperThread(self,myStepper, speed, numSteps, direction, style, limitSwitch, debugPin, lock1, lock2):
        myStepper.setSpeed(speed)
        t = threading.Thread(target=self.runStepper, args=(myStepper, numSteps, 
            direction, style, limitSwitch, debugPin, lock1, lock2))
        t.start()
        return t
