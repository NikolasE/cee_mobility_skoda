#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position
int incomingByte = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  myservo.attach(25);  // attaches the servo on pin 13 to the servo object
  Serial.begin(115200); 
}

void loop() {
  String s = Serial.readStringUntil('\n');
  if (s.length() > 0){
    myservo.writeMicroseconds(s.toInt());
  }
}
