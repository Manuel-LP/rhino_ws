#!/usr/bin/env python
# -*- coding: utf-8 -*-

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



########################################################################################################################### 
def generar_comando(motor, rango):
    # Generar el comando PD
    
        comando_pd = f"PD,{motor},{rango}\n".encode()  
        return comando_pd
  




########################################################################################################################### 
def mostrar_menu():
    while True:
        print(Fore.CYAN + "\n=========================")
        print(Fore.YELLOW + "  CONTROL DE MOTORES")
        print(Fore.CYAN + "=========================")
        print(Fore.GREEN + "Selecciona el motor que deseas mover:")
        print(Fore.WHITE + "A - Gripper")
        print("B - Motor B")
        print("C - Motor C")
        print("D - Motor D")
        print("E - Motor E")
        print("F - Motor F")
        print("q - Salir")
    
        motor = input(Fore.YELLOW + "Introduce la letra del motor o el gripper (A, B, C, D, E, F): ").strip().upper()
        if motor not in ['A','B', 'C', 'D', 'E', 'F', 'Q']:
            print(Fore.RED + "Opción inválida. Intenta de nuevo.")
            continue
        if motor == 'Q':
        	
                 break	
        if motor == 'A':
            accion_gripper = input(Fore.YELLOW + " O - para abrir el gripper\n C - para cerrarlo\n Opción: ").strip().upper()
            if accion_gripper == 'O':
                comandos_acumulados.append(b'GO\n')
            elif accion_gripper == 'C':
                comandos_acumulados.append(b'GC\n')
            else:
                print(Fore.RED + "Acción inválida para el gripper. Intenta de nuevo.")
           	
        else:
            try:
                rango = int(input(Fore.YELLOW + "Introduce el rango de movimiento (-3000 a 3000): "))
               
                if -3000 <= rango <= 3000:
                    comandos_acumulados.append(generar_comando(motor, rango))
                else:
                    print(Fore.RED + "Rango fuera de los límites permitidos. Intenta de nuevo.")
         
            except ValueError:
                print(Fore.RED + "Entrada inválida para el rango. Intenta de nuevo.")



########################################################################################################################### 
def ejecutar_comandos():
    
    print(Fore.YELLOW + "\nEjecutando todos los movimientos acumulados...\n")
    for comando in comandos_acumulados:
        if comando == b'GO' or comando == b'GC':
            ser.write(comando)  # Envía solo el comando para abrir o cerrar el gripper
        else:
            ser.write(comando)  # Envía el comando PD acumulado para motores
        time.sleep(0.1)
    ser.write(b'MI\n')  # Envía el comando MI al final para ejecutar los movimientos
    print(Fore.GREEN + "Movimientos ejecutados." + Style.RESET_ALL)



########################################################################################################################### 
def volver_punto_inicial():
    # Función para volver al punto inicial
    print(Fore.YELLOW + "\nVolviendo al punto inicial...\n")
    ser.write(b'HG\n')  # Envía el comando HG para volver al punto inicial
    print(Fore.GREEN + "Regresado al punto inicial." + Style.RESET_ALL)


########################################################################################################################### 
def leer_tecla_no_bloqueante():
    # Función para leer una tecla sin bloquear el programa
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


########################################################################################################################### 
def ejecutar_loop():
    # Función para ejecutar el loop que vuelve al punto inicial y luego ejecuta los comandos guardados
    print(Fore.YELLOW + "\nIniciando loop: Volver a la posición inicial y luego ejecutar los movimientos...\n")
    
    while True:
        # Verificar si se ha presionado la tecla 'f' sin bloquear el programa
        tecla = leer_tecla_no_bloqueante()
        if tecla == 'f':
            print(Fore.YELLOW + "Tecla 'f' presionada. Finalizando el loop." + Style.RESET_ALL)
            break
        
        # Ejecución del bucle
        volver_punto_inicial()
        time.sleep(1)
        ejecutar_comandos()
        time.sleep(1)  # Añade un pequeño retardo si es necesario para evitar que el ciclo se ejecute demasiado rápido
    
    print(Fore.GREEN + "Loop terminado." + Style.RESET_ALL)


###########################################################################################################################   
def main():
    while True: 
        print(Fore.CYAN + "\n=========================")
        print(Fore.YELLOW + "           MENU ")
        print(Fore.CYAN + "=========================")
        print(Fore.GREEN + "Selecciona la opción:")
        accion = input(" r - Para seleccionar un motor y su movimiento.\n g - Para ejecutar los movimientos\n v - Para volver al punto inicial\n l - Para iniciar el loop\n q - Para limpiar loop\n p - Para elección de modo Host o Pendant \n Ingrese opción:").strip().lower()
        print("\n")
        if accion == 'g':
            ejecutar_comandos()
            #comandos_acumulados.clear()  # Limpia la lista de comandos tras ejecutarlos
            #continuar = input(Fore.CYAN + "¿Quieres hacer más movimientos? (s/n): ").strip().lower()
            #if continuar != 's':
            #    print(Fore.YELLOW + "Finalizando el programa." + Style.RESET_ALL)
            #    break
        elif accion == 'v':
            volver_punto_inicial()
            #continuar = input(Fore.CYAN + "¿Quieres hacer más movimientos? (s/n): ").strip().lower()
            #if continuar != 's':
            #    print(Fore.YELLOW + "Finalizando el programa." + Style.RESET_ALL)
            #    break
        elif accion == 'l':
            ejecutar_loop()
            #continuar = input(Fore.CYAN + "¿Quieres hacer más movimientos? (s/n): ").strip().lower()
            #if continuar != 's':
            #    print(Fore.YELLOW + "Finalizando el programa." + Style.RESET_ALL)
            #    break
        elif accion == 'q':
            comandos_acumulados.clear()
            
        elif accion == 'p':
            print("Selecciona la opción: \n")
            modo = input(Fore.WHITE + " h - Modo host\n p - Modo pendant\n Ingrese opción:").strip().lower()

            if modo == 'h':
            	ser.write(b'TH\n')
            	print(Fore.WHITE + "Cambio a modo Host")
            elif modo == 'p':
            	ser.write(b'TX\n')
            	print(Fore.WHITE + "Cambio a modo Pendant")
            
        elif accion == 'r':
            mostrar_menu()



if __name__ == "__main__":
    main()
