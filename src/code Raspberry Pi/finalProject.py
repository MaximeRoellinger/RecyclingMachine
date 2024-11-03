# Import necessary libraries
import time  # Timer functions
import threading  # For multithreading
import RPi.GPIO as GPIO  # Raspberry Pi GPIO library
import smbus as smbus  # I2C communication library for Arduino communication
from picamera import PiCamera  # RPi Camera Module interface
import cv2  # OpenCV for image processing
from gpiozero import Servo  # Servo control library
from time import sleep  # Sleep function

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)  # Use GPIO numbering (Broadcom SOC channel)
GPIO.setwarnings(False)  # Disable GPIO warnings

# I2C communication setup with Arduino
addr = 0x08  # Arduino slave address
bus = smbus.SMBus(1)  # Use I2C bus 1 for communication

# Define GPIO pins for various inputs
generalButton = 4  # General button input
extruderMotorButton = 27  # Extruder motor control button
doorState = 25  # Door sensor input

# Define GPIO pins for various outputs
# Motor Shredder (positive and negative rotation)
shredderMotor1Positive = 26
shredderMotor2Positive = 19
shredderMotor1Negative = 13
shredderMotor2Negative = 6

# Extruder control outputs
heatExtruder = 23  # Extruder heater
motorExtruder = 24  # Extruder motor

# Interface LEDs
ledHeating = 5
ledReady = 0
ledWorking = 1
ledDoorClosed = 7
ledDoorOpen = 8
ledNoPLA = 11
ledFailure = 9

# Temperature Sensor (MAX6675) SPI configuration
cs = 16  # Chip Select
sck = 21  # Serial Clock
so = 20  # Serial Output

# Servo motor control pin
pwmServo = 18

# Input pin setup
GPIO.setup(generalButton, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(extruderMotorButton, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(doorState, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Output pin setup
for pin in [shredderMotor1Positive, shredderMotor2Positive, shredderMotor1Negative, shredderMotor2Negative,
            heatExtruder, motorExtruder, ledHeating, ledReady, ledWorking, ledDoorClosed,
            ledDoorOpen, ledNoPLA, ledFailure]:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

# Temperature sensor setup function
def set_pin(CS, SCK, SO, UNIT):
    global sck, so, unit
    sck = SCK
    so = SO
    unit = UNIT
    GPIO.setup(CS, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(SCK, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(SO, GPIO.IN)

# Read temperature from MAX6675 sensor
def read_temp(cs_no):
    GPIO.output(cs_no, GPIO.LOW)
    time.sleep(0.002)
    GPIO.output(cs_no, GPIO.HIGH)
    time.sleep(0.22)

    # Clock cycle to read 12-bit temperature
    Value = 0
    for i in range(11, -1, -1):
        GPIO.output(sck, GPIO.HIGH)
        Value += GPIO.input(so) * (2 ** i)
        GPIO.output(sck, GPIO.LOW)

    # Check for error
    GPIO.output(sck, GPIO.HIGH)
    error_tc = GPIO.input(so)
    GPIO.output(sck, GPIO.LOW)

    # Clear remaining bits
    for i in range(2):
        GPIO.output(sck, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(sck, GPIO.LOW)

    GPIO.output(cs_no, GPIO.HIGH)

    # Convert to temperature if no error
    return Value * 0.25 if error_tc == 0 else -cs_no

GPIO.cleanup()
set_pin(cs, sck, so, 1)  # Setup temperature sensor pins

# Shredder and extruder timers and status variables
global temperatureSensor, sensorShredder, startTimerShredder, countTimerShredder
global shredderNoPLA, sensorExtruder, startTimerExtruder, countTimerExtruder
global extruderNoPLA, SensorIntensity, SensorIntensityHIGH

# Initialize status variables
temperatureSensor = 0
sensorShredder, sensorExtruder, SensorIntensity = 0, 0, 0
startTimerShredder, shredderNoPLA = False, False
countTimerShredder, countTimerExtruder = 0, 0
startTimerExtruder, extruderNoPLA = False, False
SensorIntensityHIGH = False

# I2C Communication function with Arduino
def communication():
    while True:
        # Read temperature and send data to Arduino
        temperatureSensor = int(read_temp(cs))
        print("temperature =", temperatureSensor)
        data = int(WithFilament)
        bus.write_i2c_block_data(addr, 0, data)

        # Read and parse data from Arduino
        block = bus.read_i2c_block_data(addr, 0)
        n = "".join(map(chr, block)).split(";")
        print("sensor shredder:", n[0])
        sensorShredder = n[0]
        print("sensor extruder:", n[1])
        sensorExtruder = n[1]
        print("sensor Intensity", n[2])
        SensorIntensity = n[2]
        
        time.sleep(0.02)

# Camera filament detection
def getWithFilament():
    camera = PiCamera()
    camera.resolution = (1920, 1088)
    camera.framerate = 2

    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        img = frame.array
        img = cv2.resize(img, (1800, 1080))[300:1000, 500:900]
        binary = cv2.inRange(img, (0, 0, 150), (161, 161, 255))
        
        global pixelFilament, WithFilament
        pixelFilament = np.sum(binary == 0)
        WithFilament = pixelFilament / 400
        
        cv2.imshow("img", img)
        cv2.imshow("binary", binary)
        cv2.waitKey(1) & 0xFF
        raw_capture.truncate(0)
        time.sleep(0.01)

# Shredder motor control
def shredding():
    while True:
        if sensorShredder < 20:
            startTimerShredder = True
        elif sensorShredder >= 20:
            startTimerShredder = False
            countTimerShredder = 0

        # Shredder motor control logic
        if shredderNoPLA == False and doorState == 1:
            GPIO.output(shredderMotor1Positive, GPIO.HIGH)
            GPIO.output(shredderMotor2Positive, GPIO.HIGH)
            GPIO.output(ledNoPLA, GPIO.LOW)
        elif doorState == 0:
            GPIO.output(ledDoorOpen, GPIO.HIGH)

        if shredderNoPLA or SensorIntensityHIGH:
            GPIO.output(shredderMotor1Positive, GPIO.LOW)
            GPIO.output(shredderMotor2Positive, GPIO.LOW)
            SensorIntensityHIGH = False
            GPIO.output(shredderMotor1Negative, GPIO.HIGH)
            time.sleep(0.6)
            GPIO.output(shredderMotor1Negative, GPIO.LOW)

# Extruder motor control
def extrude():
    if temperatureSensor > 195:
        time.sleep(0.5)
        temperatureOK = temperatureSensor > 195
    
    if sensorExtruder < 20:
        startTimerExtruder = True
    elif sensorExtruder >= 20:
        startTimerExtruder, countTimerExtruder = False, 0
        extruderNoPLA = False

    if temperatureOK and not extruderNoPLA:
        GPIO.output(motorExtruder, GPIO.HIGH)

# Timer functions for shredder and extruder
def timerShredder():
    while True:
        if startTimerShredder:
            countTimerShredder += 1
            if countTimerShredder >= 40:
                shredderNoPLA, startTimerShredder = True, False
        time.sleep(0.1)

def timerExtruder():
    while True:
        if startTimerExtruder:
            countTimerExtruder += 1
            if countTimerExtruder >= 300:
                extruderNoPLA, startTimerExtruder = True, False
        time.sleep(0.1)

# Intensity monitoring for shredder
def getIntensity():
    iMax = 3.5
    while True:
        measure1 = sensorIntensity
        time.sleep(0.2)
        measure2 = sensorIntensity
        if measure1 > iMax and measure2 > iMax:
            SensorIntensityHIGH = True

# Servo motor control
def runServo():
    servo = Servo(pwmServo)
    while True:
        servo.min()
        sleep(0.7)
        servo.mid()
        sleep(0.7)
