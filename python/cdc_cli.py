import sys
import cmdln
from numpy import uint8
from numpy import uint32
from numpy import bitwise_xor
import numpy as np
import struct
import serial
import threading

class SerialRxEvent(object):
    def __init__(self, data):
        self.data = data
        
        
class EventHandler(object):
    def __init__(self, handler):
        self.__events = []
        self.handler = handler
    def add(self, event):
        self.__events.append(event)
    def delete(self, event):
        self.__events.remove(event)
    def fire(self):
        if len(self.__events) > 0:
            [self.handler(event.data) for event in self.__events]
            del self.__events[:]

class ChargeDeviceControl(object):
    def __init__(self):
        self.serial = serial.Serial(baudrate = 9600, timeout = 0)
        self.rxthread = None
        self.rxalive = threading.Event()
        
        self.rxevents = EventHandler(self.printRx)
        self.eventthread = None
        self.eventalive = threading.Event()
    
    def printRx(self, data):
        print ""
        print "received>%s" % data

    def eventThread(self):
        while self.eventalive.is_set():
            self.rxevents.fire()
       
    def StartRxThread(self):
        self.rxthread = threading.Thread(target=self.rxThread)
        self.rxthread.setDaemon(1)
        self.rxalive.set()
        self.rxthread.start()

    def StartEventThread(self):
        self.eventthread = threading.Thread(target=self.eventThread)
        self.eventthread.setDaemon(1)
        self.eventalive.set()
        self.eventthread.start()
        
    def rxThread(self):
        while self.rxalive.is_set():               #loop while alive event is true
            text = self.serial.read(1)          #read one, with timout
            if text:                            #check if not timeout
                n = self.serial.inWaiting()     #look if there is more to read
                if n:
                    text = text + self.serial.read(n) #get it
                event = SerialRxEvent(text)
                self.rxevents.add(event)

        
    def StopRxThread(self):
        """Stop the receiver thread, wait util it's finished."""
        if self.rxthread is not None:
            self.rxalive.clear()          #clear alive event for thread
            self.rxthread.join()          #wait until thread has finished
            self.rxthread = None

    def StopEventThread(self):
        """Stop the receiver thread, wait util it's finished."""
        if self.eventthread is not None:
            self.eventalive.clear()          #clear alive event for thread
            self.eventthread.join()          #wait until thread has finished
            self.eventthread = None
            
    def arr2str(self, arr):
        return ''.join([chr(i) for i in arr])
        
    def ul2code(self, ul):
        """Convert desired output voltage to code"""
        emp1 = 1000*5/(4095*4.5)
        return uint32(ul/emp1)


    def chksum(self, cmd, length):
        """Calculate checksum for a command"""
        a = 0
        for i in range(length - 1):
            a = bitwise_xor(a, cmd[i])
        return uint8(~a)
    
    def setvoltage(self, voltage):
        code = self.ul2code(voltage)
        bcode = [ord(i) for i in struct.pack('f', code)] #convert 32-bit word to an array of 4 bytes
        cmd = np.array([0]*8, dtype=uint8)
        cmd[0] = 1 #device number
        cmd[1] = 6 #length of the cmd
        cmd[2] = ord('D') #the cmd
        cmd[3] = bcode[0]
        cmd[4] = bcode[1]
        cmd[5] = bcode[2]
        cmd[6] = bcode[3]
        cmd[7] = self.chksum(cmd, 8)
        if self.serial.isOpen():
            self.serial.write(self.arr2str(cmd))
        else:
            print """Port is not opened"""
        
    def connect(self, port):
        self.serial.port = port
        try:
            self.serial.open()
            self.serial.flushInput()
            if self.serial.isOpen():
                print """Port %s opened""" % port
                if self.rxthread == None:
                    self.StartEventThread()
                    self.StartRxThread()
            else:
                print """Can't open %s port""" % port
        except serial.serialutil.SerialException:
            print "Can't find port %s" % port
            serial.port = '' 
       
    def disconnect(self):
        if self.rxthread is not None:
            self.StopRxThread()
            self.StopEventThread()
            self.serial.close()
            print "Port is disconnected"
        else:
            print "Port is not connected"

    def close(self):
        self.StopRxThread()
        self.StopEventThread()
        self.serial.close()
        
class ChargeDeviceCLI(cmdln.Cmdln):
    name = "charge-device"
    
    def __init__(self):
        cmdln.Cmdln.__init__(self)
        self.control = ChargeDeviceControl()
        
    def do_connect(self, subcmd, opts, port):
        """${cmd_name}: connect to specified port
        ${cmd_usage}
        """
        self.control.connect(port)
    def do_disconnect(self, subcmd, opts):
         """${cmd_name}: disconnect from conected port
         ${cmd_usage}
         """
         self.control.disconnect()
    def do_setvoltage(self, subcmd, opts, voltage):
        """${cmd_name}: set charging voltage on battery
        ${cmd_usage}
        """
        print "Set charging voltage to %s!" % voltage
        self.control.setvoltage(float(voltage))
        
    def do_getvoltage(self, subcmd, opts):
        """${cmd_name}: get charging voltage on battery
        ${cmd_usage}
        """
        print "Get charging voltage"
    def do_close(self, subcmd, opts):
        self.control.close()
        self.stop = True
        sys.exit(0)

if __name__ == "__main__":
    cli = ChargeDeviceCLI()
    cli.cmdloop()
