#include <Servo.h>
Servo myservo1;   
Servo myservo2;   
String inByte;
int pos;
int whichservo = 1;      
int go;
void setup() {
 
  myservo1.attach(9);
  myservo2.attach(8);
  Serial.begin(9600);
}

void loop()
{    
  if(Serial.available())  // if data available in serial port
    { 
    inByte = Serial.readStringUntil('\n'); // read data until newline
    pos = inByte.toFloat();   // change datatype from string to integer 
      if(whichservo == 1)   
        {
        go = map(pos, 0, 1, 0, 180);        
        myservo1.write(pos);     // move servo
        Serial.print("X position: ");  
        Serial.println(inByte);
        Serial.println(go);
        whichservo = 2;
        }
      else if(whichservo == 2)
        {       
        myservo2.write(pos);     // move servo
        Serial.print("Y position: ");  
        Serial.println(inByte);
        whichservo = 1;
        }  
    }
}
