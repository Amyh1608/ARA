import serial

ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600)

class Motor:
    def __init__(self):
        pass
    
    def forward(self):
        ser.write("forward")
        self.read()    
        
    def backward(self):
        ser.write("backward")
        self.read()
    
    def left(self):
        ser.write("left")
        self.read()
        
    def right(self):
        ser.write("right")
        self.read()
        
    def read(self):
        while(ser.in_waiting):
            print(ser.readline())
