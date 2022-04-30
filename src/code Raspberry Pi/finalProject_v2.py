
import time
import threading
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM) # use GPIO Pins 
GPIO.setwarnings(False)


import smbus as smbus # import for communication with arduino 

# Enter sensor :
magnetSensorEnter = 2 # juste un interupteur qui ouvre le circuit du moteur 


# enter actor :
motorShredder = 3
ledShredderNoPLA = 5

# Extruder sensor :
temperatureSensor = 0  # value of temperature 
laserSensorFilamentPositionHigh= 0 # get it from arduino 
laserSensorFilamentPositionLow = 0  # get it from arduino 

#Extruder Actor :
motorExtruder = 8
motorVibration = 9
ledExtruderNoPLA = 10

#Pin arduino I2C

addr = 0x08 # adresse du slave arduino 
bus = smbus.SMBus(1) # on utilise le canal I2C numero 1 


# general Pins

ledCam = 12 # surement plusieurs en serie 

timerEnd = False # bool qui devient true quand le timer de l'extruder est fini 

#Pins for communicate with MAX6675
cs = 16
sck = 21
so = 20

laserSensorShredder = 0  # value of the laser sensor reservoir shredder 
laserSensorExtruder = 0 # value of the laser sensor reservoir extruder 

temperatureSensor = 0 # temperature, resolution of 0.25 and rounded 
filamentWidth = 0   # width of the filament,get info from the raspberry pi camera and opencv

# 2 fonctions + clean up a metre dans une  classe max6675 
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


startTimer():# commencer un timer pour eteindre le moteur Extruder si pas de PLA trop longtemps 
    

 
 
set_pin(cs, sck, so, 1)

GPIO.setup(motorShredder, GPIO.OUT)
GPIO.setup(ledShredderNoPLA, GPIO.OUT)

GPIO.setup(motorExtruder, GPIO.OUT)
GPIO.setup(motorVibration, GPIO.OUT)
GPIO.setup(ledExtruderNoPLA, GPIO.OUT)


#GPIO.setup(ventilator, GPIO.OUT)
#GPIO.setup(ledCam, GPIO.OUT)

# ces 4 actions sotn réalisées dans la class max6675
#GPIO.setup(temperatureSensor, GPIO.IN)
#GPIO.setup(sckTemperatureSensor, GPIO.OUT)
#GPIO.setup(csTemperatureSensor, GPIO.OUT)
#GPIO.setup(soTemperatureSensor, GPIO.IN)



def enter():
    while(True):
        if laserSensorShredder == False:
            GPIO.output(motorShredder,GPIO.LOW)
            GPIO.output(ledShredderNoPLA,GPIO.HIGH)
        

            
        
 
# Second Method
def extrude():
    if temperatureSensor > 200 & laserSensorExtruder > 500 :  # si temperature > 200 et quil y a du PLA dans le reservoir shredder valeur de 500 a calibrer 
        GPIO.output(motorExtruder,GPIO.HIGH)
        GPIO.output(motorVibration,GPIO.HIGH)
        GPIO.output(ledExtruderNoPLA,GPIO.LOW)
        
    else if temperatureSensor > 200 & laserSensorExtruder < 500:  # si temperature > 200 et quil y a du PLA dans le reservoir shredder valeur de 500 a calibrer
        startTimer()
        GPIO.output(motorExtruder,GPIO.HIGH)
        GPIO.output(motorVibration,GPIO.HIGH)
        GPIO.output(ledExtruderNoPLA,GPIO.LOW)
        
        
    else if laserSensorExtruder < 500 & timerEnd == True:
        GPIO.output(motorExtruder,GPIO.LOW)
        GPIO.output(motorVibration,GPIO.LOW)
        GPIO.output(ledExtruderNoPLA,GPIO.HIGH)
        
           

def communication():
    while True:
        temperatureSensor = int(read_temp(cs))  # read temperature in Celcius
        print("temperature = ",temperatureSensor)
    
        block = bus.read_i2c_block_data(addr,0)  # read String data from Arduino : data1;data2;data3;...
        n = "".join(map(chr,block))
        n = n.split(";")         # split data in form of an array  [ data1 , data2, data3,...]
        print("capteur haut: "+ n[0])
        print("capteur bas: "+ n[1])
        print("capteur shredder: "+ n[2])
        print("capteur extruder: "+ n[3])
        print()

    
        bus.write_byte(addr,temperatureSensor)    # send temperature to Arduino 
        time.sleep(0.25)  # delay of 0.25 ( resolution of max6675 sensor )
    
    
# alternative provisoire :
    
   
#    try:
#        while 1:
#             temperatureSensor = read_temp(cs)
# 
 #       # print temperature
 #            print(temperatureSensor)

        # when there are some errors with sensor, it return "-" sign and CS pin number
        # in this case it returns "-22" 
        
 #            time.sleep(0.250)
 #    except KeyboardInterrupt:
  #       pass
                
        
    
        
 
#Create the Threads
threadEnter = threading.Thread(target=enter)
threadExtrude = threading.Thread(target=extrude)
threadCommunication = threading.Thread(target=communication)
 
#Start the threads
threadEnter.start()
threadExtrude.start()
threadCommunication.start()

#Join the threads ? 
#threadEnter.join()
# threadExtrude.join()

# stop thread ?
# threadEnter.stop()


# read temperature connected at CS 22
        
       
    
        


    



 
