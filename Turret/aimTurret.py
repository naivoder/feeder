import serial
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
command = input()
arduino.write(command)
print(command)
reachedPos = str(arduino.readline())
#print(reachedPos)

