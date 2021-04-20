import serial

ports = [("/dev/ttyAMA2",115200), ("/dev/ttyAMA3",9600), ("/dev/ttyAMA4",57600)]

for port,baud in ports:
    print("Cleaning: ", port, " at ", baud)
    ser = serial.Serial(port, baud)
    ser.flushInput()
    ser.flushOutput()
    ser.close()