#!/usr/bin/env python 

#****************************
# york-sim.py
# Librería de funciones para controlar el modelo de robot YORK
# utilizando scripts de Python.
# Para ser utilizada con CoppeliaSim 4.6 o superior.
# Requiere la escena york_teleop.ttt
# Código fuente e instrucciones de uso: http://github.com/cear-inacap/york-control
#
# Autor: Claudio Morales Díaz @cmoralesd
# Versión: 1.0 - mayo 2024
#*****************************

import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# variables globales
client, sim = None, None
handlers = {}

def set_ip(ip_adress):
    # inicializa un cliente de la API remota
    global client, sim
    print('conectando al robot YORK en ', ip_adress)
    client = RemoteAPIClient()
    sim = client.require('sim')
    if client != -1: 
        print('...conexión OK')
    else:
        print('...no se pudo conectar')
    return is_connected()
    
def shutdown():
    pass

def is_connected():
    return True if sim.getSimulationState()!=sim.simulation_paused else False

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
    vy = (r1+r2)*(w1-w2+w3-w4)/4
    omega_r = (a+b)*(r1+r2)*(-w1+w2+w3-w4)/(4*(a**2+b**2))
    return [vx, vy, omega_r]

def get_handlers():
    global handlers
    try:
        handlers['m1'] = sim.getObject('./york/RLmotor')
        handlers['m2'] = sim.getObject('./york/RRmotor')
        handlers['m3'] = sim.getObject('./york/FRmotor')
        handlers['m4'] = sim.getObject('./york/FLmotor')
        return True
    except:
        print('no se encuentra el robot "york" en la escena de CoppeliaSim')
        return False

def move(robot_vel):
    global handlers
    # verifica si existe conexión con 'york' en CoppeliaSim
    connected = True if handlers!={} else get_handlers()

    # calcula la cinemática inversa
    motor_vel = calc_IK(robot_vel)

    # y envía los datos al robot robot
    if connected:
        sim.startSimulation()
        sim.setJointTargetVelocity(handlers['m1'], float(motor_vel[0]))
        sim.setJointTargetVelocity(handlers['m2'], float(motor_vel[1]))
        sim.setJointTargetVelocity(handlers['m3'], float(motor_vel[2]))
        sim.setJointTargetVelocity(handlers['m4'], float(motor_vel[3]))
        return 0 # retorna 0 si el envío de datos fue exitoso
    
    return -1 # retorna -1 si hubo un error
