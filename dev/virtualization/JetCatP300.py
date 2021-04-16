import serial
import sys

class JetCatP300
    self.command = {
        "RAC" : "35000,568,1.32,11,%f,35"%self.thrust,
        "DHC" : "",
        "RTY" : "1.3,10.332,1067,5000,324,ABC", # Read Information.
    }
    self.functionCall = {
        "RHC" : rhc_function,
        "WPE" : wpe_function,
        "RFI" : rfi_function,
        "WBD" : wbd_function,
        "TCO" : tco_function
    }
    def __init__(self, port, baud):
        self.jcp_serial = serial.Serial(port, baudrate = baud,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
        self.ctrl_sig = self.pwr_sig = False
        self.thrust = 0
        self.enabled = False
        self.baud = 9600
        self.END = 13
    
    def simulate(self):
        while 1:
            data = self.jcp_serial.readline()
            if len(data) > 0:
                # process string
                values = data.split(',')
                retstr = ""
                # generate response
                if self.command.get(values[1]):
                    retstr = self.format_string(self.command[values[1]])
                elif self.functionCall.get(values[1]):
                    retstr = self.formar_string(self.functionCall[values[1]]())
                # write to serial
                self.jcp_serial.write(b'%s'%retstr)
    
    def format_string(self, resp):
        return "1,HS,OK,%s%c"%(resp,self.END)

    def rhc_function(self, value):
        health_flag = 0
        main_valve_flag = 0
        starter_valve_flag = 0
        rpm_flag = 0
        pump_flag = 0
        glowplug_flag = 0
        sensor_flag = 0
        return "%d,%d,%d,%d,%d,%d,%d"%(health_flag, main_valve_flag, 
            starter_valve_flag, rpm_flag, pump_flag, glowplug_flag, sensor_flag)
    
    def wpe_function(self, value):
        self.thrust = value
        return ""
    
    def rfi_function(self, value):
        return "2002,10000,10000,15,200,1000"

    def wbd_function(self, value):
        if value > 8:
            print ("Only value between 0 - 8 allowed.")
        self.baud = value
        return ""

    def tco_function(self, value):
        self.enabled = value
        return ""

    def __del__(self):
        self.jcp_serial.close()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise ValueError("Pass port and baud as parameter.")
    jcp = JetCatP300(sys.argv[1], int(sys.argv[2]))
    jcp.simulate()