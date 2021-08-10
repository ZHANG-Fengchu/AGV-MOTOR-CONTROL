# Code written by ZHANG Fengchu, inspired by code written by Mohammed Omar Salameh.  
# Code to be used with MDD10A.py.  
# Purpose of code is to have the motors rotate at a given speed for a certain amount of time, dependent on user given parameter values.
# Whether the motors rotates in the same direction or in opposite directions depends
# on whether the command from the user is for the vehicle to move forwards or rotate.  
import sys, tty, termios, os
import MDD10A as HBridge
import time
from time import sleep
import RPi.GPIO as GPIO
import math
from signal import signal, SIGTERM, SIGHUP, pause
from gpiozero import DistanceSensor

# Set up sensors
## IR sensor
GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.IN)

## US sensor
us_sensor = DistanceSensor(echo=20, trigger=21)
def safe_exit(signum, frame):
    exit(1)

us_threshold = 0.2
    
signal(SIGTERM, safe_exit)
signal(SIGHUP, safe_exit)

# Initialize values
max_omega_w=18.43
speedleft=0.0
speedright=0.0

# Default vehicle parameters if no user input. 
rv=0.5
rw=0.1
duty_cycle=0.1
omega_w=max_omega_w*duty_cycle
theta_v_deg=360
theta_v_rad=theta_v_deg*(math.pi)/180
rotatetime=int((theta_v_rad/omega_w)*(rv/rw))
dis=2
movetime=int(dis/(omega_w*rw))

# User raw input (for now) to determine how the user wishes for the robot to move.
def getch():
    return input() 

# The Menu 
def printscreen():
    os.system('clear')
    print("+: Right motor speed up")
    print("-: Right motor speed down")
    print(".: Left motor speed up")
    print(",: Left motor speed down")
    print("p: set vehicle parameters")
    print("f: move forwards (default 1 meter)")
    print("b: move backwards (defautl 1 meter)")
    print("r: rotate clockwise (default 90 degrees)")
    print("l: rotate anticlockwise (default 90 degrees)")
    print("q: stops the motors")
    print("x: exit")
    print("========== Current motor status ==========")
    print("current left motor duty cycle:",speedleft)
    print("current right motor duty cycle:",speedright)
    print("========== Current user parameters ==========")
    print("half of track length:",rv,"meters")
    print("radius of wheel:",rw,"meters")
    print("user duty cycle:",duty_cycle)
    print("user input wheel angular velocity:",omega_w,"radians per second")
    print("desired rotation angle in degrees:",theta_v_deg,"degrees")
    print("desired rotation angle in radians:",theta_v_rad,"radians")
    print("estimated rotation time based on current parameters:",rotatetime,"seconds")
    print("desired straight movement distance:",dis,"meters")
    print("estimated straight movement time based on current parameters:",movetime,"seconds")
    print("current ultrasound sensor distance threshold:",us_threshold,"meters")
    
# Welcome text
print("Welcome, press one of the following characters.")
printscreen()

# Main program
while True:
    # Keyboard character retrieval method. This method will save
    # the pressed key into the variable char
    char = getch()
    
    # Require user to input parameters of robot.
    if(char=="p"):
        speedleft = 0.0
        speedright = 0.0
        HBridge.setMotorLeft(speedleft)
        HBridge.setMotorRight(speedright)
        print("Please enter half of distance between the two wheels in meters.")
        rv=float(input())
        print("Please enter radius of robot wheel in meters")
        rw=float(input())
        print("Please enter wheel motor PWM duty cycle, from 0.1 to 1.0 (i.e. 10% to 100%).")
        print("Note that the absolute value of the average of the measured angular velocity of each motor at 100% duty cycle is",max_omega_w,"rads^-1.")
        duty_cycle=float(input())
        omega_w=max_omega_w*duty_cycle
        print("Please enter desired vehicle rotation in degrees, for double wheel turning.")
        theta_v_deg=float(input())
        theta_v_rad=theta_v_deg*(math.pi)/180
        rotatetime=int((theta_v_rad/omega_w)*(rv/rw))
        print("Please enter desired movement distance in meters, for straight forward and backward movement only.")
        dis=float(input())
        movetime=int(dis/(omega_w*rw))
        print("Decide Ultrasound sensor distance threshold.")
        us_threshold=float(input())
        printscreen()
    
    # The vehicle will move forwards to the user desired distance when f is pressed.
    if(char == "f"):
        speedleft = duty_cycle
        speedright = duty_cycle
        print("========== Current motor status ==========")
        print("current left motor duty cycle:",speedleft)
        print("current right motor duty cycle:",speedright)
        print("Will take:",movetime,"seconds")
        print("Press ctrl-c to terminate movement.")
        time_at_start = time.time()
        time_paused = 0
        time_elapsed = int((time.time() - time_at_start)) 
        try:     
            while(time_elapsed < movetime):
                ir_sensor=GPIO.input(25)
                time_elapsed = int((time.time() - time_at_start)) - time_paused
                if ir_sensor==1 and us_sensor.value>us_threshold:
                    HBridge.setMotorLeft(speedleft)
                    HBridge.setMotorRight(speedright)
                    print("Time elapsed:",time_elapsed,"seconds; No obstacles; US sensor value:",us_sensor.value,"meters"+ "\r", end="")
                elif ir_sensor==0 or us_sensor.value<us_threshold:
                    HBridge.setMotorLeft(0)
                    HBridge.setMotorRight(0)
                    time.sleep(1)
                    time_paused = time_paused + 1
                    print("Time elapsed:",time_elapsed,"seconds; Yes obstacle; US sensor value:",us_sensor.value,"meters; Time paused:", time_paused, "seconds" + "\r", end="")
        except KeyboardInterrupt:
            pass
        char = "q"
        print("Time's up")
        
    # The vehicle will move backwards to the user desired distance when b is pressed.
    if(char == "b"):
        speedleft = -duty_cycle
        speedright = -duty_cycle
        print("========== Current motor status ==========")
        print("current left motor duty cycle:",speedleft)
        print("current right motor duty cycle:",speedright)
        print("Will take:",movetime,"seconds")
        print("Press ctrl-c to terminate movement.")
        time_at_start = time.time()
        time_paused = 0
        time_elapsed = int((time.time() - time_at_start))
        try:
            while(time_elapsed < movetime):
                ir_sensor=GPIO.input(25)
                time_elapsed = int((time.time() - time_at_start)) - time_paused
                if ir_sensor==1 and us_sensor.value>us_threshold:
                    HBridge.setMotorLeft(speedleft)
                    HBridge.setMotorRight(speedright)
                    print("Time elapsed:",time_elapsed,"seconds; No obstacles; US sensor value:",us_sensor.value,"meters"+ "\r", end="")
                elif ir_sensor==0 or us_sensor.value<us_threshold:
                    HBridge.setMotorLeft(0)
                    HBridge.setMotorRight(0)
                    time.sleep(1)
                    time_paused = time_paused + 1
                    print("Time elapsed:",time_elapsed,"seconds; Yes obstacle; US sensor value:",us_sensor.value,"meters; Time paused:", time_paused, "seconds" + "\r", end="")
        except KeyboardInterrupt:
            pass
        char = "q"
        print("Time's up")
        
    # The vehicle will rotate clockwise to the user desired angle when r is pressed.
    if(char == "r"):
        speedleft = duty_cycle
        speedright = -duty_cycle
        print("========== Current motor status ==========")
        print("current left motor duty cycle:",speedleft)
        print("current right motor duty cycle:",speedright)
        print("Will take:",rotatetime,"seconds")
        print("Press ctrl-c to terminate movement.")
        time_at_start = time.time()
        time_paused = 0
        time_elapsed = int((time.time() - time_at_start))
        try:
            while(time_elapsed < rotatetime):
                ir_sensor=GPIO.input(25)
                time_elapsed = int((time.time() - time_at_start)) - time_paused
                if ir_sensor==1 and us_sensor.value>us_threshold:
                    HBridge.setMotorLeft(speedleft)
                    HBridge.setMotorRight(speedright)
                    print("Time elapsed:",time_elapsed,"seconds; No obstacles; US sensor value:",us_sensor.value,"meters"+ "\r", end="")
                elif ir_sensor==0 or us_sensor.value<us_threshold:
                    HBridge.setMotorLeft(0)
                    HBridge.setMotorRight(0)
                    time.sleep(1)
                    time_paused = time_paused + 1
                    print("Time elapsed:",time_elapsed,"seconds; Yes obstacle; US sensor value:",us_sensor.value,"meters; Time paused:", time_paused, "seconds" + "\r", end="")
        except KeyboardInterrupt:
            pass
        char = "q"
        print("Time's up")
    
    # The vehicle will rotate clockwise to the user desired angle when l is pressed.
    if(char == "l"):
        speedleft = -duty_cycle
        speedright = duty_cycle
        print("========== Current motor status ==========")
        print("current left motor duty cycle:",speedleft)
        print("current right motor duty cycle:",speedright)
        print("Will take:",rotatetime,"seconds")
        print("Press ctrl-c to terminate movement.")
        time_at_start = time.time()
        time_paused = 0
        time_elapsed = int((time.time() - time_at_start))
        try:
            while(time_elapsed < rotatetime):
                ir_sensor=GPIO.input(25)
                time_elapsed = int((time.time() - time_at_start)) - time_paused
                if ir_sensor==1 and us_sensor.value>us_threshold:
                    HBridge.setMotorLeft(speedleft)
                    HBridge.setMotorRight(speedright)
                    print("Time elapsed:",time_elapsed,"seconds; No obstacles; US sensor value:",us_sensor.value,"meters"+ "\r", end="")
                elif ir_sensor==0 or us_sensor.value<us_threshold:
                    HBridge.setMotorLeft(0)
                    HBridge.setMotorRight(0)
                    time.sleep(1)
                    time_paused = time_paused + 1
                    print("Time elapsed:",time_elapsed,"seconds; Yes obstacle; US sensor value:",us_sensor.value,"meters; Time paused:", time_paused, "seconds" + "\r", end="")
        except KeyboardInterrupt:
            pass
        char = "q"
        print("Time's up")
        
    # Speed up right motor
    if(char == "+"):
        speedright = speedright + 0.1
        if speedright > 1:
            speedright = 1
        HBridge.setMotorRight(speedright)
        printscreen()
        
    # Slow down right motor
    if(char == "-"):
        speedright = speedright - 0.1
        if speedright < -1:
            speedright = -1
        HBridge.setMotorRight(speedright)
        printscreen()
        
    # Speed up left motor
    if(char == "."):
        speedleft = speedleft + 0.1
        if speedleft > 1:
            speedleft = 1
        HBridge.setMotorLeft(speedleft)
        printscreen()
        
    # Slow down left motor
    if(char == ","):
        speedleft = speedleft - 0.1
        if speedleft < -1:
            speedleft = -1
        HBridge.setMotorLeft(speedleft)
        printscreen()
    
    # Stop the motors
    if(char == "q"):
        speedleft = 0
        speedright = 0
        HBridge.setMotorLeft(speedleft)
        HBridge.setMotorRight(speedright)
        printscreen()
        
    # The "x" key will break the loop and exit the program
    if(char == "x"):
        HBridge.setMotorLeft(0)
        HBridge.setMotorRight(0)
        HBridge.exit()
        print("Program Ended")
        break

    # The keyboard character variable char has to be set blank. We need
    # to set it blank to save the next key pressed by the user.
    char = ""
# End