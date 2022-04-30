
#include <AccelStepper.h>
AccelStepper bigStepper(AccelStepper::FULL2WIRE, 2, 3); // STEP pin , DIR Pin

const int SwitchClosed = 12; // Switch Fil bleu sur Pin 12    
    
int CloseSwitch=0;
bool madeSetup= false;
long leftEndStopPosition=0; // Extrémité Loin du moteur
long rightEndStopPosition=0; // Extrémité proche du moteur

// PARAMETRE
int setupSpeed= -2000; // Vitesse de mise à 0
int workingSpeed =1500; // Vitesse de fonctionnement
int range = 20000 ;// Distance entre les deux extrémités range max = 35800


// Vitesse Positive = Le chariot s'éloigne du moteur
// Vitesse Négative = Le chariot se raproche du moteur

void setup()
{
 //Serial.begin(9600);                                             // DEBUGGING
  pinMode(SwitchClosed, INPUT);
  bigStepperDecallage.setMaxSpeed(2000);
  bigStepperDecallage.setSpeed(workingSpeed);
}


void loop()
{
CloseSwitch = digitalRead(SwitchClosed); // Lecture de la valeur du Switch 0=Ouvert // 1=Fermé

//Beginnig Setup Big Stepper
if(madeSetup==false&&CloseSwitch!=1)// Setup
      {
        bigStepper.setSpeed(setupSpeed);
        bigStepper.runSpeed();
        CloseSwitch = digitalRead(SwitchClosed);
      }        
  if( CloseSwitch==1 && madeSetup == false)
    {
      leftEndStopPosition = bigStepper.currentPosition(); // Attribution de left value à la position actuelle (Current position va etre de 0)
      rightEndStopPosition = leftEndStopPosition+ range; //Attribution de la distance avec l'autre extrémité
      bigStepper.moveTo(rightEndStopPosition); 
      bigStepper.setSpeed(workingSpeed);   
      madeSetup=true;// Sortie du mode Setup
      // Serial.println("Setup Ended");                         // DEBUGGING
      }
// End Setup Big Stepper
//........................
// Control Big Stepper

  if (bigStepper.currentPosition() != bigStepper.targetPosition()&& madeSetup==true) 
    { 
      bigStepper.runSpeedToPosition();}
  
   if(bigStepper.currentPosition()>=rightEndStopPosition && madeSetup==true)
    {
      bigStepper.moveTo(leftEndStopPosition); 
      bigStepper.setSpeed(workingSpeed);   
      //Serial.println("Right Limit");                           // DEBUGGING
    }
   if(bigStepper.currentPosition()<=leftEndStopPosition && madeSetup==true)
    {
      bigStepper.moveTo(rightEndStopPosition); 
      bigStepper.setSpeed(workingSpeed);   
      //Serial.println("Left Limit");                           // DEBUGGING
     }                        
// End Control Big Stepper
  }
 
 
