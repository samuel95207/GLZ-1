import serial
import time

ser = serial.Serial("/dev/ttyACM0",115200)
while True:
    for i in range(0,180,5):
        print(i)
        print(type(str(i).encode()))
        print(type(bytes(i)))

        ser.write((str(i) + '\n').encode())
        time.sleep(0.1)

