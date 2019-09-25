import serial
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
command = bytes(input(), 'utf-8')
arduino.write(command)
print(command)
reachedPos = str(arduino.readline())
print(reachedPos)
