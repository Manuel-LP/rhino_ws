import serial
import time
from colorama import init, Fore, Style

import select

import sys
import termios
import tty
import time

# Inicializa colorama
init()

# Configura el puerto serial y los parámetros
puerto = '/dev/ttyUSB0'  # Cambia esto al puerto adecuado para tu sistema
baud_rate = 9600         # La velocidad en baudios, asegúrate de que coincida con la del dispositivo

# Abre el puerto serial con la configuración deseada
ser = serial.Serial(
    port=puerto,
    baudrate=baud_rate,
    bytesize=serial.SEVENBITS,    # 7 bits de datos
    parity=serial.PARITY_ODD,     # Paridad impar
    stopbits=serial.STOPBITS_TWO, # 2 bits de parada
    timeout=0.1                   # Tiempo de espera para operaciones de lectura
)

# Lista para almacenar los comandos
comandos_acumulados = []
accion = input("P- modos, r-rutina, v-volver").strip().lower()
if accion == 'p':
    modo = input(Fore.WHITE + " h - Modo host\n p - Modo pendant\n Ingrese opción:").strip().lower()
    if modo == 'h':
        ser.write(b'TH\n')
        print(Fore.WHITE + "Cambio a modo Host")
    elif modo == 'p':
        ser.write(b'TX\n')
        print(Fore.WHITE + "Cambio a modo Pendant")
if accion == 'v':
    print(Fore.YELLOW + "\nVolviendo al punto inicial...\n")
    ser.write(b'HG\n')  # Envía el comando HG para volver al punto inicial
    print(Fore.GREEN + "Regresado al punto inicial." + Style.RESET_ALL)
if accion == 'r':
    while True:
        print("Ejecutando")
  # Ejemplo de comando para el motor A
        #comandos_acumulados.append(b'PD,B,1115\n')  # Ejemplo de comando para el motor B
        #comandos_acumulados.append(b'PD,C,588\n')  # Ejemplo de comando para el motor C
        #comandos_acumulados.append(b'PD,D,1642\n')
        #comandos_acumulados.append(b'PD,E,1829\n')
        comandos_acumulados.append(b'PD,F,-1500\n')
        comandos_acumulados.append(b'PD,F,0\n')

        #comandos_acumulados.append(b'GC\n')
        ser.write(b'MI\n')  
        time.sleep(1) 
        print(Fore.YELLOW + "\nVolviendo al punto inicial...\n")
        ser.write(b'HG\n')  # Envía el comando HG para volver al punto inicial
        print(Fore.GREEN + "Regresado al punto inicial." + Style.RESET_ALL)
        time.sleep(1)
        if accion == 'q':
            break