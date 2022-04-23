



import time
import threading
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


GPIO.setmode(GPIO.BCM)
# Enter sensor :
laserSensorShredder = 1
magnetSensorEnter = 2 # juste un interupteur 

# enter actor :
motorShredder = 3
# stepMotorTrap = 4
ledShredderNoPLA = 5


# Extruder sensor :
temperatureSensor = 6
#laserSensorFilamentPositionHigh = 7
#laserSensorFilamentPositionLow = 15

#Extruder Actor :
#motorExtruder = 8
#motorVibration = 9
#ledExtruderNoPLA = 10

#Pin arduino I2C communication


# general Pins

#ventilator = 11
ledPixyCam = 12 # surement plusieurs en serie 

# set the pin for communicate with MAX6675
cs = 16
sck = 21
so = 20



# set pin number for communicate with MAX6675
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

    if unit == 0:
        temp = Value
    if unit == 1:
        temp = Value * 0.25
    if unit == 2:
        temp = Value * 0.25 * 9.0 / 5.0 + 32.0

    if error_tc != 0:
        return -cs_no
    else:
        return temp

GPIO.cleanup()

 
 
set_pin(cs, sck, so, 1)

#GPIO.setup(motorShredder, GPIO.OUT)
#GPIO.setup(ledShredderNoPLA, GPIO.OUT)
#GPIO.setup(temperatureSensor, GPIO.IN)
#GPIO.setup(motorExtruder, GPIO.OUT)
#GPIO.setup(motorVibration, GPIO.OUT)
#GPIO.setup(ledExtruderNoPLA, GPIO.OUT)
#GPIO.setup(ventilator, GPIO.OUT)
#GPIO.setup(ledPixyCam, GPIO.OUT)
#GPIO.setup(sckTemperatureSensor, GPIO.OUT)
#GPIO.setup(csTemperatureSensor, GPIO.OUT)

#GPIO.setup(soTemperatureSensor, GPIO.IN)


    



i=0

def enter(i):
    while(True):
        if laserSensorShredder == False:
            motorShredder

        
 
 
# Second Method
def extrude():
    for i in range(1,12121):
        print(i)
        time.sleep(0.5)
        
        
        

def readTemperature():
    try:
        while 1:
            a = read_temp(cs)

        # print temperature
            print(a)

        # when there are some errors with sensor, it return "-" sign and CS pin number
        # in this case it returns "-22" 
        
            time.sleep(0.250)
    except KeyboardInterrupt:
        pass
                
        
    
        
 
#Create the Threads
threadEnter = threading.Thread(target=enter, args=(i,))
threadExtrude = threading.Thread(target=extrude)
threadReadvalue = threading.Thread(target=readTemperature)
 
#Start the threads
threadEnter.start()
threadExtrude.start()
threadReadvalue.start()

#Join the threads
#threadEnter.join()
# threadExtrude.join()





             # read temperature connected at CS 22
        
       
    
        


    



 
