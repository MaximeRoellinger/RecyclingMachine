#include <AccelStepper.h>
#include <Wire.h>  // for I2C communication

// Define stepper motor instances
AccelStepper smallStepper(AccelStepper::FULL4WIRE, 10, 12, 11, 13); // small stepper: IN4 = 10, IN3 = 12, IN2 = 11, IN1 = 13
AccelStepper bigStepper(AccelStepper::FULL2WIRE, 4, 5); // big stepper: STEP = 4, DIR = 5
AccelStepper offsetStepper(AccelStepper::FULL2WIRE, 2, 3); // offset stepper: STEP = 2, DIR = 3

// I2C settings
#define SLAVE_ADDRESS 0x08    // I2C address for the Arduino

// Light Dependent Resistors (LDRs) for position sensing
#define LDR_TOP A0      // Top position sensor
#define LDR_BOTTOM A1   // Bottom position sensor
#define LDR_SHREDDER A2 // Shredder reservoir sensor
#define LDR_EXTRUDER A3 // Extruder reservoir sensor

// Variables for sensor readings and status
char value[25]; // Data buffer for I2C transmission
int filamentThickness = 0;
int switchStatus = 0;
bool setupComplete = false;
long leftEndStop = 0;  // Far end of offset stepper
long rightEndStop = 0; // Near end of offset stepper

const int endSwitchPin = 6; // Switch pin to detect the end position
int ldrTop, ldrBottom;      // LDR readings for top and bottom

// Parameters for stepper motor control
int bigStepperSpeed = -300;
int smallStepperSpeed = -500;
int setupSpeed = -2000;        // Speed during setup (initialization)
int operationalSpeed = 1500;   // Speed during normal operation
long travelRange = 170000;     // Distance between end stops

// Temperature settings
int targetTemperature = 200;
bool temperatureReady = false;

void setup() {
  // Set up pin modes for sensors and switches
  pinMode(LDR_TOP, INPUT);
  pinMode(LDR_BOTTOM, INPUT);
  pinMode(endSwitchPin, INPUT);

  // Initialize steppers with maximum speeds
  smallStepper.setMaxSpeed(2000);
  smallStepper.setSpeed(smallStepperSpeed);
  
  bigStepper.setMaxSpeed(2000);
  bigStepper.setSpeed(bigStepperSpeed);

  offsetStepper.setMaxSpeed(3000);
  offsetStepper.setSpeed(operationalSpeed);

  // Start I2C communication
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(sendData);  // Event for sending data to master
  Wire.onReceive(receiveData); // Event for receiving data from master

  // Serial.begin(9600); // Uncomment for debugging via Serial monitor
}

void loop() {
  // Read the switch and sensor values
  switchStatus = digitalRead(endSwitchPin);
  ldrTop = analogRead(LDR_TOP);
  ldrBottom = analogRead(LDR_BOTTOM);

  // Adjust the small stepper motor speed based on filament thickness
  if (filamentThickness < 140 && filamentThickness > 40) {
    smallStepperSpeed -= (filamentThickness - 90) * 0.5; // 1.75 mm ≈ 90 pixels
    temperatureReady = true;
  } else {
    smallStepperSpeed = -120;
  }

  // Adjust big stepper speed based on LDR difference
  if (ldrBottom > ldrTop + 7 && bigStepperSpeed > -100) {
    bigStepperSpeed -= 1;
  } else if (ldrTop >= ldrBottom + 4) {
    bigStepperSpeed += 1;
  }

  // Update stepper speeds
  smallStepper.setSpeed(smallStepperSpeed);
  smallStepper.runSpeed();
  bigStepper.setSpeed(bigStepperSpeed);
  bigStepper.runSpeed();

  // Control the offset stepper for end-stop positions
  initializeOffsetStepper();
  runOffsetStepper();
}

// Initializes the offset stepper to reach end positions
void initializeOffsetStepper() {
  if (!setupComplete && switchStatus != HIGH) {
    offsetStepper.setSpeed(setupSpeed);
    offsetStepper.runSpeed();
    switchStatus = digitalRead(endSwitchPin);
  }

  if (switchStatus == HIGH && !setupComplete) {
    leftEndStop = offsetStepper.currentPosition(); // Set left end position
    rightEndStop = leftEndStop + travelRange;      // Define right end position based on range
    offsetStepper.moveTo(rightEndStop);
    offsetStepper.setSpeed(operationalSpeed);
    setupComplete = true; // Setup is now complete
  }
}

// Runs the offset stepper between end positions
void runOffsetStepper() {
  if (offsetStepper.currentPosition() != offsetStepper.targetPosition() && setupComplete) {
    offsetStepper.runSpeedToPosition();
  }

  if (offsetStepper.currentPosition() >= rightEndStop && setupComplete) {
    offsetStepper.moveTo(leftEndStop);
    offsetStepper.setSpeed(operationalSpeed);
  }

  if (offsetStepper.currentPosition() <= leftEndStop && setupComplete) {
    offsetStepper.moveTo(rightEndStop);
    offsetStepper.setSpeed(operationalSpeed);
  }
}

// Receives data over I2C and updates filament thickness
void receiveData(int howMany) {
  while (Wire.available()) {
    filamentThickness = int(Wire.read()); // Update filament thickness
  }
}

// Sends sensor data over I2C
void sendData() {
  String dataString = String(ldrTop) + ";" + String(ldrBottom) + ";";
  dataString.toCharArray(value, 20);
  Wire.write(value); // Send data over I2C
}
