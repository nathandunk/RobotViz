#include <Servo.h>

char robot_size = 3;
int angles[3];
char buff[3];

void setup() {
  Serial.begin(9600);
  pinMode(13,OUTPUT);
}

void loop() {
  readData(angles);
  Serial.println(angles[0]);
  Serial.println(angles[1]);
  Serial.println(angles[2]);
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

