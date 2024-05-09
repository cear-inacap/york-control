#!/usr/bin/env python 

#***************************************
# york_teleop_keyboard.py
# Control de movimiento para el robot YORK utilizando el teclado.
#
# Autor: Claudio Morales Díaz @cmoralesd
# Código fuente e instrucciones de uso: http://github.com/york-control
# Versión: 1.0 - mayo de 2024
#***************************************

import time
from pynput import keyboard as kb
import york_sim as york  # york_sim para simulación
                         # york_cfv2 para el robot real


# variables globales
ip_adress = '127.0.0.1'  # la dirección ip del robot york

vx, vy, omega_r = 0, 0, 0
linear_speed = 0.1
angular_speed = 1.0
max_linear_speed = 0.4
max_angular_speed = 1.5

# mensaje inicial
info = """

---------------------------
Teclas de movimiento:
   u    i    o
   j    k    l

q/z : incrementa/decrementa la velocidad en 10%

's' para salir
"""

# despliega mensaje inicial
print(info)
time.sleep(2)

# inicializa la comunicación
york.set_ip(ip_adress)
time.sleep(1)

# on_press(), on_release(): funciones para la detección de eventos de teclado
def on_press(key):
	global vx, vy, omega_r
	global linear_speed, angular_speed, max_linear_speed, max_angular_speed
	if key==kb.KeyCode.from_char('i'): vx=1
	if key==kb.KeyCode.from_char('k'): vx=-1
	if key==kb.KeyCode.from_char('j'): omega_r=1
	if key==kb.KeyCode.from_char('l'): omega_r=-1
	if key==kb.KeyCode.from_char('u'): vy=1
	if key==kb.KeyCode.from_char('o'): vy=-1
	if key==kb.KeyCode.from_char('q'):
		if abs(linear_speed) <= max_linear_speed: linear_speed *= 1.1 
		if abs(angular_speed) <= max_angular_speed: angular_speed *= 1.1
	if key==kb.KeyCode.from_char('z'):
		linear_speed *= 0.9
		angular_speed *= 0.9
	if key == kb.KeyCode.from_char('s'):
		print('saliendo de la aplicación...')
		york.shutdown()
		return False

def on_release(key):
	global vx, vy, omega_r
	global linear_speed, angular_speed, max_linear_speed, max_angular_speed
	if key==kb.KeyCode.from_char('i'): vx=0
	if key==kb.KeyCode.from_char('k'): vx=0
	if key==kb.KeyCode.from_char('j'): omega_r=0
	if key==kb.KeyCode.from_char('l'): omega_r=0
	if key==kb.KeyCode.from_char('u'): vy=0
	if key==kb.KeyCode.from_char('o'): vy=0

# inicia lectura del teclado
key_listener = kb.Listener(on_press, on_release, suppress=True )
key_listener.start()

# envía al robot las instrucciones recibidas por teclado
while key_listener.is_alive() and york.is_connected():
	# se prepara el vector de velocidades deseadas
	motor_vel = [
		round(vx * linear_speed, 2),
		round(vy * linear_speed, 2),
		round(omega_r * angular_speed, 2)
		]
    # se envía el vector de velocidades al robot
	ret = york.move(motor_vel)

    # se imprime el resultado
	if ret != -1:
		print('vx, vy, omega_r :', motor_vel)
	else:
		print('no se pudo enviar los datos al robot')

