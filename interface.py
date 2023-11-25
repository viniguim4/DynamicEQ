import tkinter as tk
from tkinter import Scale, Label, Button
import serial
import time
from serial.tools import list_ports
import threading  
import scipy as sp
from numpy import round
#Variaveris globais
startMarker = '<'
endMarker = '>'
Separador_elemento = ';'
Separador_filtro = ','
dataStarted = False
dataBuf = ""
messageComplete = False
slider_values = [0, 0, 0, 0, 0, 0, 0]
count = 0
N_taps = 3

#========================

def  agrupar(lista, original):
    global N_taps
    for valor in lista[:N_taps//2+1]:
        original.append(round(valor, 3))
    return original
def CalcularCoeficientes(slider_values):
    global N_taps
    coef = []
    freq_passa_baixas = slider_values[1]
    freq_passa_faixas = slider_values[3:5]
    freq_passa_altas = slider_values[6]

    coef.append(slider_values[0])
    LP = sp.signal.firwin(N_taps, freq_passa_baixas, pass_zero="lowpass", fs =650)
    coef = agrupar(LP, coef)

    coef.append(slider_values[2])
    BP = sp.signal.firwin(N_taps, freq_passa_faixas, pass_zero="bandpass", fs =8000)
    coef = agrupar(BP, coef)

    coef.append(slider_values[5])
    HP = sp.signal.firwin(N_taps, freq_passa_altas, pass_zero="highpass", fs =40050)
    coef = agrupar(HP, coef)
    return coef

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

def SendData(stringToSend):
    
        # this adds the start- and end-markers before sending
    global startMarker, endMarker, serialPort
    
    stringWithMarkers = (startMarker)
    stringWithMarkers += stringToSend
    stringWithMarkers += (endMarker)
    
    serialPort.write(stringWithMarkers.encode('utf-8')) # encode needed for Python3


#==================

def ReciveData():

    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

    if serialPort.inWaiting() > 0 and messageComplete == False:
        line = serialPort.readline().decode('utf-8').strip()

        for caracter in line:
            if dataStarted == True:
                if caracter != endMarker:
                    dataBuf = dataBuf + caracter
                else:
                    dataStarted = False
                    messageComplete = True
            elif caracter == startMarker:
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
        msg = ReciveData()
        if not (msg == 'XXX'): 
            print(msg)


def FormataDados(slider_values):
    global N_taps, Separador_elemento, Separador_filtro
    envio = ""
    atualizar = CalcularCoeficientes(slider_values)
    for i in range(0, len(atualizar)):
        if(i == N_taps//2+2 or i == (2*N_taps)//2+3):
            envio += Separador_filtro
        envio += str(atualizar[i])
        if(i != len(atualizar)-1):
            envio += Separador_elemento
    return envio

def Comunicar2():
    global count, slider_values
    prevTime = time.time()
    while True:
                # check for a reply
        arduinoReply = ReciveData()
        if not (arduinoReply == 'XXX'):
            print("   Recebido: %s" % arduinoReply)
            print()
            
            # send a message at intervals
        if time.time() - prevTime > 1.0:
            envio = FormataDados(slider_values)
            SendData(envio)
            print(str(count) + "  Enviado: " + str(envio) )
            prevTime = time.time()
            count += 1

def Comunicar():
    global count, slider_values
    envio = FormataDados(slider_values)
    SendData(envio)
    print(str(count) + "  Enviado: " + str(envio) )
    count += 1
    arduinoReply = ReciveData()
    if not (arduinoReply == 'XXX'):
        print("   Recebido: %s" % arduinoReply)
        print()
    else:
        print("   Recebido: nada")        

    
#==================== Funções da Interface ====================
def update_label(self):
    time.sleep(0.11)
    slider_values[0] = slider1.get()
    slider_values[1] = slider2.get()
    slider_values[2] = slider3.get()
    slider_values[3] = slider4.get()
    slider_values[4] = slider7.get()
    slider_values[5] = slider5.get()
    slider_values[6] = slider6.get()

    label.config(text=f"Valores: {slider_values[0]}, {slider_values[1]}, {slider_values[2]}, {slider_values[3]}, {slider_values[4]}, {slider_values[5]}, {slider_values[6]}")

    try:
        threading.Thread(target=Comunicar).start()
    except Exception as e:
        print(f"Error: {e}")
    

def on_button_click():
    # Create a new thread for sending data
    Comunicar()
    """try:
        threading.Thread(target=Comunicar).start()
    except Exception as e:
        print(f"Error: {e}")"""
    
#==================== Create the main window====================
setupSerial(19200, select_serial_port())
root = tk.Tk()
root.title("Coletar Parametros")

# Create seven sliders with increased length
slider1 = Scale(root, from_=0, to=100, orient="horizontal", label="Ganho", length=400) 
slider2 = Scale(root, from_=20, to=320, orient="horizontal", label="Frequência de Corte", length=400)
slider3 = Scale(root, from_=0, to=100, orient="horizontal", label="Ganho", length=400) 
slider4 = Scale(root, from_=320, to=720, orient="horizontal", label="Frequência de Corte Inferior", length=400) 
slider7 = Scale(root, from_=720, to=3200, orient="horizontal", label="Frequência de Corte Superior", length=400)
slider5 = Scale(root, from_=0, to=100, orient="horizontal", label="Ganho", length=400) 
slider6 = Scale(root, from_=3200, to=20000, orient="horizontal", label="Frequência de Corte", length=400)

# Create labels for slider pairs with increased font size
label_pair1 = Label(root, text="Passa Baixas", font=("Arial", 18))
label_pair2 = Label(root, text="Passa Faixas", font=("Arial", 18))
label_pair3 = Label(root, text="Passa Alta", font=("Arial", 18))

# Create a label to display the current values of the sliders
label = Label(root, text="Valores: ", font=("Arial", 12))
button = Button(root, text="Enviar Dados", command=on_button_click)

# Set the command to be called when a slider is moved
slider1.config(command=update_label)
slider2.config(command=update_label)
slider3.config(command=update_label)
slider4.config(command=update_label)
slider5.config(command=update_label)
slider6.config(command=update_label)
slider7.config(command=update_label)

# Pack the widgets into the window
label_pair1.pack()
slider1.pack()
slider2.pack()

label_pair2.pack()
slider3.pack()
slider4.pack()
slider7.pack()

label_pair3.pack()
slider5.pack()
slider6.pack()

label.pack()

button.pack()
# Start the main loop
root.mainloop()
