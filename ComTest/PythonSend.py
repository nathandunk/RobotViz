import serial
import time

angle1 = 130
angle2 = 100
angle3 = 115

ser = serial.Serial('COM6', 9600)
time.sleep(5)

ser.write('075');
ser.write('115');
ser.write('060');


# ser.write(str(angle1));
# ser.write(str(angle2));
# ser.write(str(angle3));