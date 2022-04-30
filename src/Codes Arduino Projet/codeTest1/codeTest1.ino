
#include <AccelStepper.h>
AccelStepper smallStepper(AccelStepper::FULL4WIRE, 6, 10, 9, 11); // IN4 = 6 IN3 = 9  IN2 = 10 IN1 = 11 si mauvais sens alors inverser les branchements
AccelStepper bigStepper(AccelStepper::FULL2WIRE, 4, 5); // STEP pin , DIR Pin
AccelStepper bigStepperDecallage(AccelStepper::FULL2WIRE, 2, 3); // STEP pin , DIR Pin

int CloseSwitch = 0;
bool endSetup = false;
long leftEndStopPosition = 0; // Extrémité Loin du moteur
long rightEndStopPosition = 0; // Extrémité proche du moteur

const int SwitchClosed = 12; // Switch Fil bleu sur Pin 12

// PARAMETRE
int setupSpeed = -2000; // Vitesse de mise à 0
int workingSpeed = 1500; // Vitesse de fonctionnement
int range = 20000 ;// Distance entre les deux extrémités range max = 35800


// Vitesse Positive = Le chariot s'éloigne du moteur
// Vitesse Négative = Le chariot se raproche du moteur


void setup()
{

  pixy.init();
    
  smallStepper.setMaxSpeed(300);
  smallStepper.setSpeed(200);


  bigStepper.setMaxSpeed(300);
  bigStepper.setSpeed(200);

  pinMode(SwitchClosed, INPUT);
  bigStepperDecallage.setMaxSpeed(2000);
  bigStepperDecallage.setSpeed(workingSpeed);



}
void loop()
{

  pixy.ccc.getBlocks();

  Serial.println(pixy.ccc.blocks[0].m_height) ;

  smallStepperSpeed = map 
 
  smallStepper.setSpeed(200);
  smallStepper.runSpeed();


  bigStepper.setSpeed(300);
  bigStepper.runSpeed();


  CloseSwitch = digitalRead(SwitchClosed); // Lecture de la valeur du Switch 0=Ouvert // 1=Fermé

  if (endSetup == false) {
    setupMotorDecallage() ;
  }
  
  runMotorDecallage();

  

}

void setupMotorDecallage() { // avance vers la droite jus'au switch, puis definit les positions droite et gauche

  if (CloseSwitch != 1) {
    bigStepperDecallage.setSpeed(setupSpeed);
    bigStepperDecallage.runSpeed();
    CloseSwitch = digitalRead(SwitchClosed);
  }

  if ( CloseSwitch == 1) {
    leftEndStopPosition = bigStepperDecallage.currentPosition(); // Attribution de left value à la position actuelle (Current position va etre de 0)
    rightEndStopPosition = leftEndStopPosition + range; //Attribution de la distance avec l'autre extrémité
    bigStepperDecallage.moveTo(rightEndStopPosition);
    bigStepperDecallage.setSpeed(workingSpeed);
    endSetup = true; // Sortie du mode Setup
  }
}



void runMotorDecallage() {  // fait des allez reour position droite <=> position gauche en boucle

  if (bigStepperDecallage.currentPosition() >= rightEndStopPosition && endSetup == true)
  {
    bigStepperDecallage.moveTo(leftEndStopPosition);
    bigStepperDecallage.setSpeed(workingSpeed);

    //Serial.println("Right Limit");
  }
  if (bigStepperDecallage.currentPosition() <= leftEndStopPosition && endSetup == true) {
    bigStepperDecallage.moveTo(rightEndStopPosition);
    bigStepperDecallage.setSpeed(workingSpeed);
    //Serial.println("Left Limit");
  }
  if (bigStepperDecallage.currentPosition() != bigStepperDecallage.targetPosition() && endSetup == true) {
    bigStepperDecallage.runSpeedToPosition();
  }
}
