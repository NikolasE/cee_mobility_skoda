#include <Arduino.h>
#undef max
#undef min
#include <map>

int PINS[] = {21, 5, 6, 9, 10, 11, 12, 13};
int TIMEOUT = 2000;

void move_stop()
{
  for(int i=0; i<8; i++)
  {
    digitalWrite(PINS[i], HIGH);
  }
}

void move_left()
{
  for(int i=0; i<8; i++)
  {
    digitalWrite(PINS[i], HIGH);
  }
  digitalWrite(PINS[1-1], LOW);
  digitalWrite(PINS[4-1], LOW);
}

void move_right()
{
  for(int i=0; i<8; i++)
  {
    digitalWrite(PINS[i], HIGH);
  }
  digitalWrite(PINS[2-1], LOW);
  digitalWrite(PINS[6-1], LOW);
}

void move_up()
{
  for(int i=0; i<8; i++)
  {
    digitalWrite(PINS[i], HIGH);
  }
  digitalWrite(PINS[2-1], LOW);
  digitalWrite(PINS[5-1], LOW);
}

void move_down()
{
  for(int i=0; i<8; i++)
  {
    digitalWrite(PINS[i], HIGH);
  }
  digitalWrite(PINS[3-1], LOW);
  digitalWrite(PINS[4-1], LOW);
}

void setup() {
  // set channels to output, inactive
  for(int i=0; i<8; i++)
  {
    pinMode(PINS[i], OUTPUT);
  }

  move_stop();
  
  while(!Serial);
  Serial.begin(115200);
}

uint32_t last_cmd_stamp = 0;
void loop()
{
  uint32_t now = millis();
  if(Serial.available())
  {
    last_cmd_stamp = now;
    char c = Serial.read();
    switch(c)
    {
      case 'l':
      case 'L':
        move_left();
        break;
      case 'r':
      case 'R':
        move_right();
        break;
      case 'u':
      case 'U':
        move_up();
        break;
      case 'd':
      case 'D':
        move_down();
        break;
      case 's':
      case 'S':
      default:
        move_stop();
        break;
    }
  }

  if(now > last_cmd_stamp + TIMEOUT)
  {
    move_stop();
  }

  // move_left();
  // delay(1000);
  // move_up();
  // delay(1000);
  // move_right();
  // delay(1000);
  // move_down();
  // delay(1000);
}