#include <Servo.h>

char robot_size = 3;
int angles[] = {0,0,0};
char buff[3]; // input from serial port

Servo myservo1;  // create servo object to control a servo
Servo myservo2;
Servo myservo3;
// twelve servo objects can be created on most boards

int current_pos[] = {90,90,90};    // variable to store the servo position
int pos_d;

void setup() {
  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(11);
  Serial.begin(9600);
  myservo1.write(current_pos[0]);
  myservo1.write(current_pos[1]);
  myservo1.write(current_pos[2]);
}

void loop() {

  readData(angles);
  
  current_pos[0] = GoTo(myservo1,angles[0],current_pos[0]);
  current_pos[1] = GoTo(myservo2,angles[1],current_pos[1]);
  current_pos[2] = GoTo(myservo3,angles[2],current_pos[2]);
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

  while (current_position*dir < desired_position*dir)
  {
    current_position += 1*dir;
    servo_handle.write(current_position);
    Serial.println(current_position);
    delay(20);
  }
  return current_position;
}

void readData(int angles_vector[])
{
  int angle[3];
  for(int i=0;i<3;i++)
  {
    while(Serial.available() == 0){} // wait for serial input
    Serial.readBytes(buff,3); // read the input
    angles_vector[i] = (buff[0]-'0')*100+(buff[1]-'0')*10+(buff[2]-'0');
  }
}


