
//# include <Thread.h>

#include <AccelStepper.h>

#include <Servo.h>

AccelStepper smallStepper(AccelStepper::FULL4WIRE, 6, 10, 9, 11); // IN4 = 6 IN3 = 9  IN2 = 10 IN1 = 7 et anciennement 11 ,  si mauvais sens alors inverser les branchements
AccelStepper bigStepper(AccelStepper::FULL2WIRE, 4, 5); // STEP pin , DIR Pin
AccelStepper bigStepperDecallage(AccelStepper::FULL2WIRE, 2, 3); // STEP pin , DIR Pin

Servo myservo;
#include <Wire.h>  // pou I2C
#define SLAVE_ADRESS 0x08    // definit l'adresse slave 

#define LDR1 A1  // bas 
#define LDR0 A0  // haut
#define LDR3 A2  // shredder reservoir
#define LDR4 A3  // extruder reservoir

char value [25]; // la data pour envoyer

int CloseSwitch = 0;
bool endSetup = false;
long leftEndStopPosition = 0; // Extrémité Loin du moteur
long rightEndStopPosition = 0; // Extrémité proche du moteur

const int SwitchClosed = 12; // Switch Fil bleu sur Pin 12

//int pos = 0; // position du servomoteur

int haut; // capteur haut

int bas; // capteur bas

//Thread myThreadServo = Thread(); // thread qui lance le servomoteur

// PARAMETRE


// MOTEUR DECALLAGE
// Vitesse Positive = Le chariot s'éloigne du moteur
// Vitesse Négative = Le chariot se raproche du moteur


// A CALIBRER

int speedBigMotor = -300;
int speedSmallMotor = -500 ;
int setupSpeed = -2000; // Vitesse de mise à 0
int workingSpeed = 1500; // Vitesse de fonctionnement
long range = 160000 ;// Distance entre les deux extrémités // 160 000 pas mal

int temperature = 200 ;
bool temperatureOK = false;
int withFilament = 0;

void setup()
{


  pinMode(LDR1, INPUT);
  pinMode(LDR0, INPUT);

  smallStepper.setMaxSpeed(500);
  smallStepper.setSpeed(speedSmallMotor);


  bigStepper.setMaxSpeed(2000);
  bigStepper.setSpeed(speedBigMotor);

  pinMode(SwitchClosed, INPUT);  // bouton pour le stepmotor decallage
  bigStepperDecallage.setMaxSpeed(3000);
  bigStepperDecallage.setSpeed(workingSpeed);

  Wire.begin(SLAVE_ADRESS); // adresse de l'arduino pour le raspberry pi
  Wire.onRequest(sendData);

  Wire.onReceive(receiveEvent);  // lorsque le raspberry pi envoie une info



  Serial.begin(9600);


}


void setupMotorDecallage() {

  if (endSetup == false && CloseSwitch != 1)
  {
    bigStepperDecallage.setSpeed(setupSpeed);
    bigStepperDecallage.runSpeed();
    CloseSwitch = digitalRead(SwitchClosed);
  }
  if ( CloseSwitch == 1 && endSetup == false)
  { bigStepperDecallage.setCurrentPosition(-20000);
    bigStepperDecallage.setSpeed(workingSpeed);
    leftEndStopPosition = bigStepperDecallage.currentPosition(); // Attribution de left value à la position actuelle (Current position va etre de 0)
    //  Serial.println(leftEndStopPosition);
    rightEndStopPosition = leftEndStopPosition + range; //Attribution de la distance avec l'autre extrémité
    //  Serial.println(rightEndStopPosition);

    bigStepperDecallage.moveTo(rightEndStopPosition);
    bigStepperDecallage.setSpeed(workingSpeed);
    endSetup = true; // Sortie du mode Setup

  }
}


void runMotorDecallage() {

  if (bigStepperDecallage.currentPosition() != bigStepperDecallage.targetPosition() && endSetup == true)
  {
    bigStepperDecallage.runSpeedToPosition();
  }

  if (bigStepperDecallage.currentPosition() >= rightEndStopPosition && endSetup == true)
  {
    bigStepperDecallage.moveTo(leftEndStopPosition);
    bigStepperDecallage.setSpeed(workingSpeed);
    //Serial.println("Right Limit");
  }
  if (bigStepperDecallage.currentPosition() <= leftEndStopPosition && endSetup == true)
  {
    bigStepperDecallage.moveTo(rightEndStopPosition);
    bigStepperDecallage.setSpeed(workingSpeed);
    //Serial.println("Left Limit");
  }
}


void loop()
{
  CloseSwitch = digitalRead(SwitchClosed); // Lecture de la valeur du Switch 0=Ouvert // 1=Fermé

  haut = analogRead(LDR1);

  bas = analogRead(LDR0);


  Serial.println("haut :");
  Serial.println(haut);

  Serial.println(" ");

   Serial.println("bas :");
  Serial.println(bas);
  Serial.println(" ");

  Serial.println("epaisseur :");
  Serial.println(withFilament);

  Serial.println(" ");

  if (withFilament < 140 & withFilament > 40) {
    speedSmallMotor = speedSmallMotor - (withFilament - 90) * 0.5 ; //  1.75 mm <=> 90 pixels 
    
    temperatureOK = true;
  }
  else {   speedSmallMotor = -120 ;               }

  smallStepper.setSpeed(speedSmallMotor);

  if ( bas > haut + 7 & speedBigMotor > -100 ) {

    speedBigMotor = speedBigMotor - 1 ;

  }

  else if ( haut >= bas + 4 ) {

    speedBigMotor = speedBigMotor + 1;
  }

  //Serial.println(speedBigMotor);

if (temperatureOK == true) {

 smallStepper.runSpeed();
  bigStepper.runSpeed();
  setupMotorDecallage();
  runMotorDecallage();
  //
}

}


void receiveEvent(int howMany) {
  while ( Wire.available())
  {
    //ignorer la valeur 0
    withFilament =  int(Wire.read()); // receive byte as a character

    //   if (temperatureTest > 0) {         // tricks pour eviter la valeur 0 qui s'intercale TROUVER SOLUTION
    //   temperature = temperatureTest;
    //   Serial.println(temperature);
  }
}

void sendData() {

  String filamentPosHigh   = String(haut );// avec n le nombre de chiffres apres la virgule
  String filamentPosLow  = String(bas );    // = String(filamentPositionLow, n )
  // String capteurPLAShredder = String(capteurPLAShredd ) ;    // = String(capteur ..., n )
  // String capteurPLAExtruder  = String(capteurPLAExtr )  ;      // = String(capteur .., n )


  // String filamentPosHigh = String(haut);//  avec ,n sachant que n est le nombre de chiffres apres la virgule
  // String filamentPosLow  = String(bas) ;     // = String(filamentPositionLow, n )

  // String capteurPLAShredd =  String(capteurPLAShredd);
  // String capteurPLAExtr  =   String(capteurPLAExtr) ;


  String DataTotal = (filamentPosHigh + ";" + filamentPosLow + ";"); //+ capteurPLAShredd + ";" + capteurPLAExtr + ";");

  DataTotal.toCharArray(value, 20);
  Wire.write(value);
}
