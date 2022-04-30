

#include <Wire.h>  // pou I2C
#define SLAVE_ADRESS 0x08    // definit l'adresse slave 

#define LDR1 A1  // composante photorésistance sur la pin A1
#define LDR2 A0
#define LDR3 A2
#define LDR4 A3


#include <Pixy2.h>

#include <AccelStepper.h>   // Accel steper pour les 2 step moteurs 

char value [25];
int temperature ;

int haut;    // valeur photoresistances
int bas;
int diff;    // difference des deux

int   capteurPLAShredd;  // valeur photoresistance reservoir shredder
int capteurPLAExtr;  // valeur photoresistance reservoir Extruder
String enableStepMotors ; // valeur a recevoir du Raspi

AccelStepper smallStepper(AccelStepper::FULL4WIRE, 6, 10, 9, 11);   // IN4 = 6 IN3 = 9  IN2 = 10 IN1 = 11 si mauvais sens alors inverser les branchements
AccelStepper bigStepper(AccelStepper::FULL2WIRE, 4, 5);  // STEP pin , DIR Pin

Pixy2 pixy;

void setup() {


  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);
  pinMode(LDR3, INPUT);
  pinMode(LDR4, INPUT);

  pixy.init();

  smallStepper.setMaxSpeed(300);
  smallStepper.setSpeed(200);


  bigStepper.setMaxSpeed(300);
  bigStepper.setSpeed(200);    //-100 c'etait pas mal    entre - 70 quand fil en haut  et -110 quand fil en bas

//  Serial.begin(9600);
  Wire.begin(SLAVE_ADRESS);
  Wire.onRequest(sendData);
  Wire.onReceive(receiveEvent);

}


void loop() {


  haut = analogRead(LDR1);
  bas = analogRead(LDR2);
  diff = haut - bas;
  capteurPLAShredd = analogRead(LDR3);
  capteurPLAExtr = analogRead(LDR4);


  // pixy.ccc.getBlocks();
  // filament = map(pixy.ccc.blocks[0].m_height, 1, , 1.5 ,2  );

  // Serial.println(pixy.ccc.blocks[0].m_height) ;
  // Serial.println("");

//  if (temperature > 28 ) {


    smallStepper.setSpeed(200);
    smallStepper.runSpeed();


    bigStepper.setSpeed(500);
    bigStepper.runSpeed();        // vitesse ici en fonction de la position du filament (haut et bas )
    //  bigStepper.runSpeed();

 // }
  // else if (enableStepMotors == 2) {

  //    smallStepper.stop();
  //  bigStepper.stop();
  //smallStepper.disableOutputs();
  // bigStepper.disableOutputs();



  //  }


}


void sendData() {

  String filamentPosHigh   = String(haut );// avec n le nombre de chiffres apres la virgule
  String filamentPosLow  = String(bas );    // = String(filamentPositionLow, n )
  String capteurPLAShredder = String(capteurPLAShredd ) ;    // = String(capteur ..., n )
  String capteurPLAExtruder  = String(capteurPLAExtr )  ;      // = String(capteur .., n )


  // String filamentPosHigh = String(haut);//  avec ,n sachant que n est le nombre de chiffres apres la virgule
  // String filamentPosLow  = String(bas) ;     // = String(filamentPositionLow, n )

  // String capteurPLAShredd =  String(capteurPLAShredd);
  // String capteurPLAExtr  =   String(capteurPLAExtr) ;


  String DataTotal = (filamentPosHigh + ";" + filamentPosLow + ";" + capteurPLAShredd + ";" + capteurPLAExtr + ";");

  DataTotal.toCharArray(value, 20);
  Wire.write(value);
  delay(250);


}

void receiveEvent(int howMany)
{
  while ( Wire.available())
  {
    //ignorer la valeur 0
    int temperatureTest = int( Wire.read()); // receive byte as a character
    if (temperatureTest > 0) {         // tricks pour eviter la valeur 0 qui s'intercale TROUVER SOLUTION
      temperature = temperatureTest;
   //   Serial.println(temperature);
    }


  }

}
// print the character
