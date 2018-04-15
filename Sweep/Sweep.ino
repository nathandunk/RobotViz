#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int current_pos = 90;    // variable to store the servo position
int pos_d;

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
  myservo.write(current_pos);
}

void loop() {
  while(!Serial.available() ){}
  int pos_d = Serial.parseInt();
  Serial.println(pos_d);
  current_pos = GoTo(myservo,pos_d,current_pos);
}

int GoTo(Servo servo_handle, int desired_position, int current_position)
{
  int dir;
  
  if (current_position < desired_position)
  {
    dir = 1;
  }
  else if (current_position > desired_position)
  {
    dir = -1;
  }
  else
  {
    return current_position;
  }

  while (current_position*dir <
  desired_position*dir)
  {
    current_position += 1*dir;
    servo_handle.write(current_position);
    Serial.println(current_position);
    delay(10);
  }
  return current_position;
}



