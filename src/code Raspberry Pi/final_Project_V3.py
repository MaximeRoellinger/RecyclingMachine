# code 3D PRM V3

import time # enables use of timer 
import threading  # enables to create threads for simultaneous actions 
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM) # use GPIO Pins 
GPIO.setwarnings(False)
import smbus as smbus # import for communication with arduino 
from picamera import PiCamera # Provides a Python interface for the RPi Camera Module
import cv2 # OpenCV library for image processing
from gpiozero import Servo
from time import sleep


addr = 0x08 # adresse du slave arduino 
bus = smbus.SMBus(1) # on utilise le canal I2C numero 1


# INPUTS :

generalButton = 4
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
ledHeathing = 5
ledReady = 0
ledWorking = 1
ledDoorClosed = 7
ledDoorOpen = 8
ledNoPLA = 11
ledFailure = 9


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



#Set up inputs :

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



set_pin(cs, sck, so, 1) # set Pins for the Temperature Sensor MAX6675, 1 for the unit we want to use : 1 is for Celcius 




global temperatureSensor
temperatureSensor = 0



# SHREDDER

global sensorShredder  # variable analogique du capteur de presence au niveau du reservoir shredder 
sensorShredder = 0
 
global startTimerShredder       # variable permetant au timer de commencer a compter 
startTimerShredder = False


global counTimerShredder     #variable du nombre de secondes ecoulées 
counTimerShredder = 0

global shredderNoPLA       # variable qui sactive quand le timer est fini on considere qu'il bn'y a plus de PLA
shredderNoPLA = False




#EXTRUDER                 pareil pour le timer extruder 

global sensorExtruder
sensorExtruder = 0

global startTimerExtruder
startTimerextruder = False 

global countTimerExtruder
counTimerShredder = 0

global extruderNoPLA
extruderNoPLA = False



global SensorIntensity
SensorIntensity = 0

global SensorIntensityHIGH
SensorIntensityHIGH=False  # variables declenchée quand le capteur d'intensité est trop haut pendfant un certain temps (donc le shredder est bloqué)





def communication():
    
    #global SensorIntensityHIGH (celui la est dans un thread solo )
    
    while True:
        
        temperatureSensor = int(read_temp(cs))# read temperature in Celcius
        print("temperature = ",temperatureSensor)
        data = int(WithFilament)
        bus.write_i2c_block_data(addr,0, data) #  send temperature to Arduino
        
        
        
       
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
    camera.framerate = 2
    # Generates a 3D RGB array and stores it in rawCapture
    raw_capture = PiRGBArray(camera, size=(1920, 1088))
    
    
    
    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    
        img = frame.array
        img = cv2.resize(img, (1800,1080))
        img = img[300:1000, 500:900] # refocus the image to save ressources 
        binary = cv2.inRange(img, (0,0,150),(161,161,255))
        
        global pixelFilament
        global WithFilament 
        pixelFilament = np.sum(binary==0)  # get number of black pixels 
        WithFilament = pixelFilament / 400  
    
        # Display the frame using OpenCV
        cv2.imshow("img", img)
        cv2.imshow("binaire", binary)
     
        # Wait for keyPress for 1 millisecond
        key = cv2.waitKey(1) & 0xFF
     
        # Clear the stream in preparation for the next frame
        raw_capture.truncate(0)
        
        time.sleep(0.01)
    
    




def shredding():
    
    while True :
         
        
        if sensorShredder < 20: # a plus que 20 on considere qu'un objet est passé
            global startTimerShredder
            startTimerShredder = True
            
            
            #demarer le thread timer shredder en mettant la variable qui debloque le timer a 1   variable en global 
            
        if sensorShredder >= 20:
            #reset la variable du timer qui compte et metre la varuiable qui autorise le timer a 0   variables en global a definir puis attribuer valeur
            global startTimerShredder
            startTimerShredder = False
            global countTimerShredder
            countTimerShredder = 0
        
        
        if shredderNoPLA == False & doorState==1 :
            GPIO.output(shredderMotor1Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Positiv, GPIO.HIGH)
            GPIO.output(shredderMotor1Positiv, GPIO.HIGH)  # motor +
            GPIO.output(ledNoPLA, GPIO.LOW)
            
        if doorState==0 :
            GPIO.output(shredderMotor1Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Positiv, GPIO.LOW)
            GPIO.output(shredderMotor1Positiv, GPIO.LOW)
            GPIO.output(ledDoorOpen, GPIO.HIGH)
            
        
        if shredderNoPLA == True:   # cest quoi ca 
            GPIO.output(shredderMotor1Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Positiv, GPIO.LOW)
            GPIO.output(shredderMotor1Positiv, GPIO.LOW)
            GPIO.output(ledNoPLA, GPIO.HIGH)
            
        if sensorIntensityHIGH == True:  # stop and goes backward
            
            sensorIntensityHIGH = False
            GPIO.output(shredderMotor1Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Positiv, GPIO.LOW)
            GPIO.output(shredderMotor1Positiv, GPIO.LOW)
            
            time.sleep(0.5)
            
            GPIO.output(shredderMotor1Negativ, GPIO.HIGH)
            GPIO.output(shredderMotor2Negativ, GPIO.HIGH)
            GPIO.output(shredderMotor2Positiv, GPIO.LOW)
            GPIO.output(shredderMotor1Positiv, GPIO.LOW)
            
            time.sleep(0.6)
            
            
            GPIO.output(shredderMotor1Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Negativ, GPIO.LOW)
            GPIO.output(shredderMotor2Positiv, GPIO.LOW)
            GPIO.output(shredderMotor1Positiv, GPIO.LOW)
            
    
    
    
def timerShredder():
  
    while True:
        if startTimerShredder == True:
           countTimerShredder = countTimerShredder+1
           time.sleep(1)
        
        if countTimerShredder >= 40:
           global shredderNoPLA
           shredderNoPLA=True
           global startTimerShredder
           startTimerShredder = False
           
           
        time.sleep(0.1)
                   


    
def extrude():
    
    if temperatureSensor > 195 & temperatureOK == False :
        time.sleep(0.5)
        if temperatureSensor > 195:
            temperatureOK = True 
            
        
    if temperature < 180 & temperature > 175:
        temperatureOK = False
          
     
    if sensorExtruder < 20: # a plus que 20 on considere qu'un objet est passé
        global startTimerExtruder
        startTimerExtruder = True
                 
    #demarer le thread timer shredder en mettant la variable qui debloque le timer a 1   variable en global 
            
    if sensorExtruder >= 20:
        #reset la variable du timer qui compte et metre la varuiable qui autorise le timer a 0   variables en global a definir puis attribuer valeur
        global startTimerExtruder
        startTimerExtruder = False
        global countTimerExtruder
        countTimerExtruder = 0
        global extruderNoPLA
        extruderNoPLA = False 
        
        
    if temperatureOK == True & extruderNoPLA == False  :
        
        GPIO.output(motorExtruder, GPIO.HIGH)
         
    if temperatureOK == False:
        
        GPIO.output(motorExtruder, GPIO.LOW)
        
    if extruderNoPLA == True:
        GPIO.output(motorExtruder, GPIO.LOW)
        
        
    
     
    
def timerExtruder():
    while True:
        if startTimerExtruder == True:
           countTimerExtruder = countTimerShredder+1
           time.sleep(1)
        
        if countTimerExtruder >= 300:
           global extruderNoPLA
           extruderNoPLA=True
           global startTimerExtruder
           startTimerExtruder = False
           
           
        time.sleep(0.1)
    
    
    
    
def getIntensity(): # lire intensité delai lire intensité , si les deux sont hauts alors variable est True
    
iMax = 3.5

    while true :
        
        measure1 = sensorIntensity
    
        time.sleep(0.2)
    
        measure2 = sensorIntensity
    
        if (measure1 > iMax & measure2 > iMax):
            global sensorIntensityHIGH
            sensorIntensityHIGH = true
       
    
    
    
def runServo():
    
    servo = Servo(pwmServo)
    
    while True:
        
         servo.min()
         sleep(0.7)
         servo.mid()
         sleep(0.7)
    




