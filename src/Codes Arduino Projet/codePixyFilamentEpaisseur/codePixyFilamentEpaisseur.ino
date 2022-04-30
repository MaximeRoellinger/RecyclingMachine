#include <Pixy2.h>

// This is the main Pixy object 
Pixy2 pixy;




void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  pixy.init();
}

void loop() {
  // put your main code here, to run repeatedly:

    pixy.ccc.getBlocks();
    
// filament = map(pixy.ccc.blocks[0].m_height, 1, , 1.5 ,2  );
    
 Serial.println(pixy.ccc.blocks[0].m_height) ;
 Serial.println("");
 delay(1000);

}
