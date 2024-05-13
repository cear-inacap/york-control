#!/usr/bin/env python 

#***************************************
# york_teleop_joystick.py
# Control de movimiento para el robot YORK utilizando un joystick genérico.
#
# Autor: Claudio Morales Díaz @cmoralesd
# Código fuente e instrucciones de uso: http://github.com/cear-inacap/york-control
# Versión: 1.0 - mayo 2024
#***************************************

import time
from cfv_gamepad import cfv_gamepad
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
Conecte el joystick para continuar...

Presiona <control> + c para salir
"""

# despliega mensaje inicial
print(info)
time.sleep(2)

# inicializa la comunicación
york.set_ip(ip_adress)
time.sleep(1)

# crea un objeto joystick
joy = cfv_gamepad()

# envía al robot las instrucciones recibidas desde el joystick
while york.is_connected():
	# obtenemos los datos desde el joystick
	# valores analógicos se escalan al rango -1 a 1
	# valores digitales se expresan en formato booleano
	vx = (128.0 - joy.LeftJoystickY)/127
	vy = (128.0 - joy.RightJoystickX)/127
	omega_r = (128.0 - joy.LeftJoystickX)/127
	speed_up = bool(joy.B)
	speed_down = bool(joy.A)

	# se ajusta la velocidad en incrementos de 1%
	if speed_up:
		if abs(linear_speed) <= max_linear_speed: linear_speed *= 1.01
		if abs(angular_speed) <= max_angular_speed: angular_speed *= 1.01
	if speed_down:
		if abs(linear_speed) >= 0.01: linear_speed *= 0.99
		if abs(angular_speed) >= 0.01: angular_speed *= 0.99

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

