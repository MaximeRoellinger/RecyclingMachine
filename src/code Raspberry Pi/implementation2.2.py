import time
import threading
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM) # use GPIO Pins 
GPIO.setwarnings(False)
import threading
from picamera.array import PiRGBArray # Generates a 3D RGB array
from picamera import PiCamera # Provides a Python interface for the RPi Camera Module
import time # Provides time-related functions
import cv2 # OpenCV library
import numpy as np

import smbus as smbus # import for communication with arduino
addr = 0x08 # adresse du slave arduino 
bus = smbus.SMBus(1) # on utilise le canal I2C numero 1

heathExtruder = 24
motorExtruder = 25

cs = 16  
sck = 21
so = 20




generalButton = 22
extruderMotorButon = 27
doorState = 25

# OUTPUTS :

#MotorShredder (positive sense of rotation //  negative sens of rotation) 
shredderMotor1Positiv = 26
shredderMotor2Positiv = 19
shredderMotor1Negativ = 13
shredderMotor2Negativ = 6
#Extruder
heathExtruder = 24
motorExtruder = 23
#Interface :

ledFailure = 9

ledHeathing =  7 
ledReady =  8
ledWorking = 1             
ledDoorClosed = 0
ledDoorOpen = 11 
ledNoPLA = 5 




global pixelFilament
pixelFilament = 0

global WithFilament
WithFilament = 0

global temperatureSensor
temperatureSensor = 0


def set_pin (CS, SCK, SO, UNIT):
    global sck
    sck= SCK
    global so
    so = SO
    global unit
    unit = UNIT
    
    GPIO.setup(CS, GPIO.OUT, initial = GPIO.HIGH)
    GPIO.setup(SCK, GPIO.OUT, initial = GPIO.HIGH)
    GPIO.setup(SO, GPIO.IN)

def read_temp(cs_no):
    
    GPIO.output(cs_no, GPIO.LOW)
    time.sleep(0.002)
    GPIO.output(cs_no, GPIO.HIGH)
    time.sleep(0.22)

    GPIO.output(cs_no, GPIO.LOW)
    GPIO.output(sck, GPIO.HIGH)
    time.sleep(0.001)
    GPIO.output(sck, GPIO.LOW)
    Value = 0
    for i in range(11, -1, -1):
        GPIO.output(sck, GPIO.HIGH)
        Value = Value + (GPIO.input(so) * (2 ** i))
        GPIO.output(sck, GPIO.LOW)

    GPIO.output(sck, GPIO.HIGH)
    error_tc = GPIO.input(so)
    GPIO.output(sck, GPIO.LOW)

    for i in range(2):
        GPIO.output(sck, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(sck, GPIO.LOW)

    GPIO.output(cs_no, GPIO.HIGH)

    if unit == 1:
        temp = Value * 0.25

    if error_tc != 0:
        return -cs_no
    else:
        return temp

GPIO.cleanup()

set_pin(cs, sck, so, 1) # set up Pins of temperature sensor 







def getWithdFilament():
    
    # Initialize the camera
    camera = PiCamera()
    # Set the camera resolution
    camera.resolution = (1920, 1088)
    # Set the number of frames per second
    camera.framerate = 2
    # Generates a 3D RGB array and stores it in rawCapture
    raw_capture = PiRGBArray(camera, size=(1920, 1088))
    
    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        image = frame.array
    
 
        img = frame.array
        img = cv2.resize(img, (1800,1080))
        img = img[300:1000, 500:900] # refocus the image to save ressources 
        binary = cv2.inRange(img, (0,0,150),(161,161,255))
        
        global pixelFilament
        global WithFilament 
        pixelFilament = np.sum(binary==0)
        WithFilament = pixelFilament / 400
        #print("number of white pixels = ",pixelFilament)
        print("high = ", WithFilament)
        print(" ")
    
        # Display the frame using OpenCV
        #cv2.imshow("img", img)
        #cv2.imshow("binaire", binary)
     
        # Wait for keyPress for 1 millisecond
        key = cv2.waitKey(1) & 0xFF
     
        # Clear the stream in preparation for the next frame
        raw_capture.truncate(0)
        
        time.sleep(0.1)
        

def communication():
    while True:
        global temperatureSensor
        temperatureSensor = int(read_temp(cs))# read temperature in Celcius
        print("temperature = ",temperatureSensor)
        
        data = int(WithFilament)
        
        bus.write_byte(addr, data)     # send temperature to Arduino 
        #time.sleep(0.25)  # delay of 0.25 ( resolution of max6675 sensor )


    
#Create the Threads
threadCamera = threading.Thread(target=getWithdFilament)
threadCommunication = threading.Thread(target=communication)
 
#Start the threads
threadCamera.start()
threadCommunication.start()

GPIO.setup(heathExtruder, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(motorExtruder, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledHeathing, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledReady, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledDoorClosed, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledDoorOpen, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledNoPLA, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledWorking, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledFailure, GPIO.OUT,initial = GPIO.LOW)


GPIO.output(heathExtruder, GPIO.LOW)
GPIO.output(motorExtruder, GPIO.LOW)


GPIO.output(ledHeathing, GPIO.HIGH)
GPIO.output(ledReady, GPIO.HIGH)
GPIO.output(ledWorking, GPIO.HIGH)
GPIO.output(ledDoorOpen, GPIO.HIGH)
GPIO.output(ledDoorClosed, GPIO.HIGH)
GPIO.output(ledNoPLA, GPIO.HIGH)
GPIO.output(ledFailure, GPIO.HIGH)




















