import serial
import time
from serial.tools import list_ports

startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False

#========================
#========================
    # the functions
def get_serial_ports():
    ports = list_ports.comports()
    port_names = [port.device for port in ports]
    return port_names

def select_serial_port():
    port_names = get_serial_ports()
    for port_name in port_names:
        if(port_name != 'COM5' and port_name != 'COM6'):
            return port_name
        
def setupSerial(baudRate, serialPortName):
    
    global  serialPort
    
    serialPort = serial.Serial(port= serialPortName, baudrate = baudRate, timeout=0, rtscts=True)

    print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))

    waitForArduino()

#========================

def sendToArduino(stringToSend):
    
        # this adds the start- and end-markers before sending
    global startMarker, endMarker, serialPort
    
    stringWithMarkers = (startMarker)
    stringWithMarkers += stringToSend
    stringWithMarkers += (endMarker)

    serialPort.write(stringWithMarkers.encode('utf-8')) # encode needed for Python3


#==================

def recvLikeArduino():

    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

    if serialPort.inWaiting() > 0 and messageComplete == False:
        x = serialPort.read().decode("utf-8") # decode needed for Python3
        
        if dataStarted == True:
            if x != endMarker:
                dataBuf = dataBuf + x
            else:
                dataStarted = False
                messageComplete = True
        elif x == startMarker:
            dataBuf = ''
            dataStarted = True
    
    if (messageComplete == True):
        messageComplete = False
        return dataBuf
    else:
        return "XXX" 

#==================

def waitForArduino():

    # wait until the Arduino sends 'Arduino is ready' - allows time for Arduino reset
    # it also ensures that any bytes left over from a previous message are discarded
    
    print("Waiting for Arduino to reset")
     
    msg = ""
    while msg.find("Arduino is ready") == -1:
        msg = recvLikeArduino()
        if not (msg == 'XXX'): 
            print(msg)



#====================
#====================
    # the program


setupSerial(115200, select_serial_port())
count = 0
prevTime = time.time()
while True:
            # check for a reply
    arduinoReply = recvLikeArduino()
    if not (arduinoReply == 'XXX'):
        print("   Recebido: %s" % arduinoReply)
        print()
        
        # send a message at intervals
    if time.time() - prevTime > 1.0:
        envio = "0;0.06409854584519187;0.8718029083096164;0;0.06446671826569138;0.8816707422179915;0;-0.014170981833061933;0.971658036333876"
        sendToArduino(envio)
        print(str(count) + "  Enviado: " + envio)
        prevTime = time.time()
        count += 1