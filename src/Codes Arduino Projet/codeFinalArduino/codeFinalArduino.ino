#include <Wire.h>  // pou I2C
#define SLAVE_ADRESS 0x08    // definit l'adresse slave 

#define CapteurFilamentHaut A1  // composante photorésistance sur la pin A1
#define CapteurFilamentBas A0    // verifier si inverser haut et bas 

#define CapteurPLAShredder A2
#define CapteurPLAExtruder A3

#include <Pixy2.h>

#include <AccelStepper.h>   // Accel steper pour les 2 step moteurs 


AccelStepper smallStepper(AccelStepper::FULL4WIRE, 6, 10, 9, 11);   // IN4 = 6 IN3 = 9  IN2 = 10 IN1 = 11 si mauvais sens alors inverser les branchements
AccelStepper bigStepper(AccelStepper::FULL2WIRE, 3, 5);  // STEP pin , DIR Pin

Pixy2 pixy;

char value [20];  // data avec 20 char

int haut;    // valeur photoresistances
int bas;
int diff;    // difference des deux

int   capteurPLAShredd;  // valeur photoresistance reservoir shredder
int capteurPLAExtr;  // valeur photoresistance reservoir Extruder
String enableStepMotors ;

void setup() {

  Serial.begin(9600);

  Wire.begin(SLAVE_ADRESS);
  Wire.onRequest(sendData);  // lorsque le maitre demande, envoie data
    Wire.onReceive(receiveEvent);


  // definit photoresistance comme analog input
  pinMode(CapteurFilamentBas, INPUT);
  pinMode(CapteurFilamentHaut, INPUT);
  pinMode(CapteurPLAShredder, INPUT);
  pinMode(CapteurPLAExtruder, INPUT);

  //initialise Pixy Cam
  pixy.init();

  //initialise stepper motors

  smallStepper.setMaxSpeed(2000);
 


  bigStepper.setMaxSpeed(2000);     //-100 c'etait pas mal    entre - 70 quand fil en haut  et -110 quand fil en bas



}

void loop() {

  haut = analogRead(CapteurFilamentBas);
  bas = analogRead(CapteurFilamentHaut);
  diff = haut - bas;

  Serial.println("haut : " + haut);
  Serial.println("bas : " + bas);
  Serial.println("");
  Serial.println("difference : " + diff);


  capteurPLAShredd = analogRead(CapteurPLAShredder);
  capteurPLAExtr = analogRead(CapteurPLAExtruder);




  pixy.ccc.getBlocks();
  // filament = map(pixy.ccc.blocks[0].m_height, 1, , 1.5 ,2  );

  Serial.println(pixy.ccc.blocks[0].m_height) ;
  Serial.println("");

  if (enableStepMotors == 1 ) {


    smallStepper.enableOutputs();
     bigStepper.enableOutputs() ;

    smallStepper.setSpeed(300);     // vitesse ici en fonction de l'epaisseur du filament
    bigStepper.setSpeed(300);

    smallStepper.runSpeed();        // vitesse ici en fonction de la position du filament (haut et bas )
    bigStepper.runSpeed();

  }
  else if (enableStepMotors == 2) {

    smallStepper.stop();
    bigStepper.stop();
    smallStepper.disableOutputs();
    bigStepper.disableOutputs();



  }

}

void sendData() {
  // voir pour envoyer seulement la difference ?
 // String filamentPosHigh = String(haut);//  avec ,n sachant que n est le nombre de chiffres apres la virgule
//  String filamentPosLow  = String(bas) ;     // = String(filamentPositionLow, n )
//  String capteurPLAShredd = String(capteurPLAShredd);    // = String(capteur ..., n )
//  String capteurPLAExtr  = String(capteurPLAExtr) ;      // = String(capteur .., n )


  
  String filamentPosHigh  = "5" ;     //  = String(filamentPositionHigh, n ) avec n le nombre de chiffres apres la virgule
  String filamentPosLow  = "10" ;     // = String(filamentPositionLow, n )
  String capteurPLAShredd = "true" ;    // = String(capteur ..., n )
  String capteurPLAExtr  = "false" ;      // = String(capteur .., n )

  //crer un string comportant toutes les variables separées par des ";" attention ne pas depasser 20 char ou alors augmenter la taille attention a prendre tous sauf le dernier string
  String DataTotal = (filamentPosHigh + ";" + filamentPosLow + ";" + capteurPLAShredd + ";" + capteurPLAExtr + ";");

  DataTotal.toCharArray(value, 20);
  Wire.write(value);  // on envoie la data
  delay(250);

}

void receiveEvent(int howMany)
{
  while( Wire.available()) 
  {
    String myString = String( Wire.read()); // receive byte as a character
    Serial.println(myString);
  
    // print the character
  }
  
}
