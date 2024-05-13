#!/usr/bin/env python 

#****************************
# york-cfv2.py
# Librería de funciones para controlar el robot YORK con tarjeta CFV2
# utilizando scripts de Python.
# Requiere que el robot real aloje el script 'york_cfv2.ino'
# Código fuente e instrucciones de uso: http://github.com/cear-inacap/york-control
#
# Autor: Claudio Morales Díaz @cmoralesd
# Versión: 1.0 - mayo 2024
#*****************************

import time
from pymodbus.client import ModbusTcpClient

# variables globales

client = None

def set_ip(ip_adress):
    # inicializa un cliente de la API remota
    global client
    print('conectando al robot YORK en ', ip_adress)
    client = ModbusTcpClient(ip_adress)
    client.connect()  
    if client.connect() : 
        print('...conexión OK')
        return True
    else:
        print('...no se pudo conectar')
        return False

def shutdown():
    client.close()

def is_connected():
    return client.connect

def calc_IK(robot_vel):
    # robot_vel es un arreglo de 3 elementos numéricos
    vx, vy, omega_r = robot_vel[0], robot_vel[1], robot_vel[2]
    r1, r2 = 0.097, 0.01  # radios de la rueda Mecanum
    a, b = 0.125, 0.105   # distancias a los ejes de rueda
    omega_1 = (-omega_r*(a*r2+b*(r1+r2))+(r1+r2)*vy+vx*(r1+r2))/(r1**2+2*r1*r2+r2**2)
    omega_2 = (omega_r*(a*r2+b*(r1+r2))-(r1+r2)*vy+vx*(r1+r2))/(r1**2+2*r1*r2+r2**2)
    omega_3 = (omega_r*(a*r2+b*(r1+r2))+(r1+r2)*vy+vx*(r1+r2))/(r1**2+2*r1*r2+r2**2)
    omega_4 = (-omega_r*(a*r2+b*(r1+r2))-(r1+r2)*vy+vx*(r1+r2))/(r1**2+2*r1*r2+r2**2)
    return [omega_1, omega_2, omega_3, omega_4]

def calc_DK(motor_vel):
    # motor_vel es un arreglo de 4 elementos numéricos
    w1, w2, w3, w4 = motor_vel[0], motor_vel[1], motor_vel[2], motor_vel[3]
    r1, r2 = 0.097, 0.01  # radios de la rueda Mecanum
    a, b = 0.125, 0.105   # distancias a los ejes de rueda
    vx = (r1+r2)*(w1+w2+w3+w4)/4
    vy = r2*(w1-w2+w3-w4)/4
    omega_r = (a*r2+b*(r1+r2))*(-w1+w2+w3-w4)/(4*(a**2+b**2))
    return [vx, vy, omega_r]


def move(robot_vel):
    # calcula la cinemática inversa
    motor_vel = calc_IK(robot_vel)

    # y envía los datos al robot
    if is_connected():
        try:
            # Las velocidades de los motores se envían a 4 registros Modbus: 40101 al 40104
            # Los valores de velocidad 0-100% en ambos sentidos de giro
            # se envían escalados a 16 bits, según:
            #       0 = -100% de velocidad (giro en sentido de reversa)
            #   16384 = 0% de velocidad, motor detenido
            #   32768 = 100% de velocidad (giro en sentido positivo)
            client.write_registers(40101, [int(val * 16384/10 + 16384) for val in motor_vel], slave=1)
            return 0 # retorna 0 si el envío de datos fue exitoso
        except: pass
            
    return -1 # retorna -1 si hubo un error
