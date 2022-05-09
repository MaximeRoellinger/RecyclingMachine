import time
import threading
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM) # use GPIO Pins 
GPIO.setwarnings(False)

import smbus as smbus # import for communication with arduino
addr = 0x08 # adresse du slave arduino 
bus = smbus.SMBus(1) # on utilise le canal I2C numero 1 

heathExtruder = 8
motorExtruder = 25

# temperature Sensor pins 
cs = 16  
sck = 21
so = 20


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


set_pin(cs, sck, so, 1) # set up Pins of temperature sensor 
temperatureSensor= 0
GPIO.setup(heathExtruder, GPIO.OUT)
GPIO.setup(motorExtruder, GPIO.OUT)

GPIO.output(heathExtruder, GPIO.HIGH)
GPIO.output(motorExtruder, GPIO.LOW)

#

while True:
    global temperature
    temperature = read_temp(cs)
    print(temperature)
    temperature = int(temperature)
    if temperatureSensor > 200:
        GPIO.output(motorExtruder, GPIO.HIGH)
  #  elif temperatureSensor < 185:
   #     GPIO.output(motorExtruder, GPIO.LOW)
        
    bus.write_byte(addr,temperature)
    
    time.sleep(0.3)
        
        








