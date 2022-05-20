# code 3D PRM V 3.2

import time # enables use of timer 
import threading  # enables to create threads for simultaneous actions 
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM) # use GPIO Pins 
GPIO.setwarnings(False)
import smbus as smbus # import for communication with arduino 
from picamera import PiCamera # Provides a Python interface for the RPi Camera Module
import cv2 # OpenCV library for image processing
from gpiozero import Servo # library to control Servomotor
from time import sleep 


addr = 0x08 # adresse of the slave (arduino) 
bus = smbus.SMBus(1) # we use canal I2C number 1


# INPUTS :

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
heathExtruder = 23
motorExtruder = 24

#Interface :
ledHeathing = 7
ledReady = 8
ledWorking = 1
ledDoorClosed = 0
ledDoorOpen = 11
ledNoPLA = 5
ledFailure = 9


#I2C connection with arduino :

serialData = 2
serialClock = 3

#Temperature Sensor MAX6675 :

cs = 16  
sck = 21
so = 20

# ServoMotor

pwmServo = 18



#Set up inputs as  :

GPIO.setup(generalButton, GPIO.IN, pull_up_down =GPIO.PUD_DOWN)
GPIO.setup(extruderMotorButon, GPIO.IN, pull_up_down =GPIO.PUD_DOWN)
GPIO.setup(doorState, GPIO.IN, pull_up_down =GPIO.PUD_DOWN)

# Set up output :

GPIO.setup(shredderMotor1Positiv, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(shredderMotor2Positiv, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(shredderMotor1Negativ, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(shredderMotor2Negativ, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(heathExtruder, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(motorExtruder, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledHeathing, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledReady, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledWorking, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledDoorClosed, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledDoorOpen, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledNoPLA, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledFailure, GPIO.OUT,initial = GPIO.LOW)

# Set up I2C :
bus = smbus.SMBus(1) # on utilise le canal I2C numero 1 donc les pins 16,20,21

def set_pin (CS, SCK, SO, UNIT):
    global sck
    sck= SCK
    global so
    so = SO
    global unit
    unit = UNIT
    
    GPIO.setup(CS, GPIO.OUT, initial = GPIO.HIGH)
    GPIO.setup(SCK, GPIO.OUT, initial = GPIO.LOW)
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



#set_pin(cs, sck, so, 1) # set Pins for the Temperature Sensor MAX6675, 1 for the unit we want to use : 1 is for Celcius 


global temperatureSensor
temperatureSensor = 0


# SHREDDER

global sensorShredder  # variable analogique du capteur de presence au niveau du reservoir shredder 
sensorShredder = 0
 


#EXTRUDER                 

global sensorExtruder  # variable analogique du capteur de presence au niveau du reservoir extruder 
sensorExtruder = 0

# ICURRENT

global sensorIntensity
sensorIntensity = 0

global sensorIntensityHIGH
sensorIntensityHIGH=False


global withFilament
withFilament = 0 



def communication():
    
    
    while True:
        
 
        
        if GPIO.input(doorState) == 1:
            GPIO.output(ledDoorOpen, GPIO.LOW)
            GPIO.output(ledDoorClosed, GPIO.HIGH)
            
        if GPIO.input(doorState) == 0:
            GPIO.output(ledDoorClosed, GPIO.LOW)
            GPIO.output(ledDoorOpen, GPIO.HIGH)
            
        
  
        global temperatureSensor
        temperatureSensor = int(read_temp(cs))# read temperature in Celcius
        
        data = int(withFilament)
        # si le bouton general est off alors data = 0
        bus.write_byte(addr,withFilament) #  send temperature to Arduino
        

        block = bus.read_i2c_block_data(addr,0)  # read String data from Arduino : data1;data2;data3;...
        n = "".join(map(chr,block))
        n = n.split(";")         # split data in form of an array  [ data1 , data2, data3,...]
        print("sensor shredder: "+ n[0])
        
        global sensorShredder 
   
        sensorShredder = n[0]
        print("sensor extruder: "+ n[1]) # utiliser des variables global true ou false ex si capteur > 20 True,   le capteur de courant devra avoir un thread seul
        
        global sensorExtruder
        sensorExtruder = n[1]
        
        print("sensor Intensity "+n[2])
        global sensorIntensity
        sensorIntensity = n[2]
        
        print("")
        time.sleep(0.02)  # le moins de delai possible 




def getWithdFilament():
    
    # Initialize the camera
    camera = PiCamera()
    # Set the camera resolution
    camera.resolution = (1920, 1088)
    # Set the number of frames per second
    camera.framerate = 5
    # Generates a 3D RGB array and stores it in rawCapture
    raw_capture = PiRGBArray(camera, size=(1920, 1088))
    
    while True:
        
        
        for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
           img = frame.array
           img = cv2.resize(img, (1800,1080))
           img = img[300:1000, 500:900] # refocus the image to save ressources 
           binary = cv2.inRange(img, (0,0,150),(161,161,255))
        
           
           global withFilament 
           pixelFilament = np.sum(binary==0)  # get number of black pixels 
           WithFilament = pixelFilament / 400  
    
           # Display the frame using OpenCV
           #cv2.imshow("img", img)
           #cv2.imshow("binaire", binary)
     
           # Wait for keyPress for 1 millisecond
           key = cv2.waitKey(1) & 0xFF
     
           # Clear the stream in preparation for the next frame
           raw_capture.truncate(0)
        
           time.sleep(0.01)
    
        


def shredding():
    
    while True :
        
        
        
        if doorState==1 & GPIO.input(generalButton)==1  :
            GPIO.output(shredderMotor1Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Positiv, GPIO.HIGH)
            GPIO.output(shredderMotor1Positiv, GPIO.HIGH)  # motor +
            GPIO.output(ledNoPLA, GPIO.LOW)
            
        else :
            
            GPIO.output(shredderMotor1Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Positiv, GPIO.LOW)
            GPIO.output(shredderMotor1Positiv, GPIO.LOW)
           
            
            
        if sensorIntensityHIGH == True:  # stop and goes backward
            
            sensorIntensityHIGH = False
            GPIO.output(shredderMotor1Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Positiv, GPIO.LOW)
            GPIO.output(shredderMotor1Positiv, GPIO.LOW)
            
            time.sleep(0.5)
            
            
            
            # executer 10 fois avec le delai 0.05 pour que le moteur se stop meme lorsqu'il fait marche arriere 
            for i in range(10):
                
                
                if doorState==1 & GPIO.input(generalButton)==1 :
                    GPIO.output(shredderMotor1Negativ, GPIO.HIGH)
                    GPIO.output(shredderMotor2Negativ, GPIO.HIGH)
                    GPIO.output(shredderMotor2Positiv, GPIO.LOW)
                    GPIO.output(shredderMotor1Positiv, GPIO.LOW)
                
                else:
                    GPIO.output(shredderMotor1Negativ, GPIO.HIGH)
                    GPIO.output(shredderMotor2Negativ, GPIO.HIGH)
                    GPIO.output(shredderMotor2Positiv, GPIO.LOW)
                    GPIO.output(shredderMotor1Positiv, GPIO.LOW)
                    
                    
                    
            
                time.sleep(0.05)
            
            
            GPIO.output(shredderMotor1Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Positiv, GPIO.LOW)
            GPIO.output(shredderMotor1Positiv, GPIO.LOW)
            
         


    
def extrude():
    

    temperatureOK = False    
    while 1:
    
        time.sleep(0.1)
    
    
    
    
        
        
        print("temperature : ", temperatureSensor)
    
        if GPIO.input(generalButton)==1:
            GPIO.output(heathExtruder, GPIO.HIGH)
            
        else:
            GPIO.output(heathExtruder, GPIO.LOW)
            
    
    
        if temperatureSensor > 190 & temperatureOK == False :
            time.sleep(0.2)
            if temperatureSensor > 190:
                temperatureOK = True 
            
        
        if temperatureSensor < 180 & temperatureOK == True:
            time.sleep(0.2)
            if temperatureSensor < 180:
                temperatureOK = False 
        
          
        
        
        if temperatureOK == True & GPIO.input(extruderMotorButon) == 1 & GPIO.input(generalButton)==1:
        
            GPIO.output(motorExtruder, GPIO.HIGH)
          
            GPIO.output(ledReady, GPIO.LOW)
            GPIO.output(ledHeathing, GPIO.LOW)
            GPIO.output(ledWorking, GPIO.HIGH)
            
            
        
        
         
        elif GPIO.input(generalButton)==1 :
            if temperatureOK == True:
                GPIO.output(motorExtruder, GPIO.LOW)
                GPIO.output(ledWorking, GPIO.LOW)
                GPIO.output(ledHeathing, GPIO.LOW)
                GPIO.output(ledReady, GPIO.HIGH)
                
            elif temperatureOK == False: 
                GPIO.output(motorExtruder, GPIO.LOW)
                GPIO.output(ledWorking, GPIO.LOW)
                GPIO.output(ledReady, GPIO.LOW)
                GPIO.output(ledHeathing, GPIO.HIGH)
                
                
        
        
            
            
        else: 
            GPIO.output(motorExtruder, GPIO.LOW)
            GPIO.output(ledWorking, GPIO.LOW)
            GPIO.output(ledReady, GPIO.LOW)
            GPIO.output(ledHeathing, GPIO.LOW)
        
        
    
    
        


    
def getIntensity(): 
    
    iMax = 3.5 # max Intensity allowed, if I ismore then go backward

    while true :
        
        measure1 = sensorIntensity
    
        time.sleep(0.2)
    
        measure2 = sensorIntensity
    
        if (measure1 > iMax & measure2 > iMax):
            global sensorIntensityHIGH
            sensorIntensityHIGH = True
            
            
        
    
    
    
    
def runServo():
    
    servo = Servo(pwmServo)
    
    while True:
        
         servo.min()
         sleep(0.7)
         servo.mid()
         sleep(0.7)
    
   
   
    


#Set up inputs :


GPIO.setmode(GPIO.BCM) # use GPIO Pins 

set_pin(cs, sck, so, 1)

GPIO.setmode(GPIO.BCM) # use GPIO Pins

GPIO.setup(generalButton, GPIO.IN, pull_up_down =GPIO.PUD_DOWN)
GPIO.setup(extruderMotorButon, GPIO.IN, pull_up_down =GPIO.PUD_DOWN)
GPIO.setup(doorState, GPIO.IN, pull_up_down =GPIO.PUD_DOWN)

# Set up output :

GPIO.setup(shredderMotor1Positiv, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(shredderMotor2Positiv, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(shredderMotor1Negativ, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(shredderMotor2Negativ, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(heathExtruder, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(motorExtruder, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledHeathing, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledReady, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledWorking, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledDoorClosed, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledDoorOpen, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledNoPLA, GPIO.OUT,initial = GPIO.LOW)
GPIO.setup(ledFailure, GPIO.OUT,initial = GPIO.LOW)
    
    
# lancer tous les threads 
    
    
#Create the Threads
threadCamera = threading.Thread(target=getWithdFilament)
threadCommunication = threading.Thread(target=communication)
threaddGetIntensity = threading.Thread(target=getIntensity)
threadServo = threading.Thread(target=runServo)
threadExtruder = threading.Thread(target=extrude)
threadShredder = threading.Thread(target=shredding)





threadCommunication.start()
threadExtruder.start()




    
    

    
    

    
    
    





    
    






