#define LDR1 A1  // composante photorésistance sur la pin A1
#define LDR2 A0

int haut;

int bas;
int diff;

void setup() {
   // initialise la communication avec le PC
   Serial.begin(9600);

   // initialise les broches
   pinMode(LDR1, INPUT);
   pinMode(LDR2, INPUT);

}

void loop() {

   haut = analogRead(LDR1);
  bas = analogRead(LDR2);

diff = haut-bas;

Serial.println("haut : "+haut);
Serial.println("bas : "+bas);
Serial.println("");
Serial.println("");

Serial.println("difference : "+diff);


   delay(300);
}
