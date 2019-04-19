#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import UInt8
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import visao_module




# Funções de Leitura
bumper = None
laser = [10]*360
def scaneou(dado):
	global laser
	laser = np.array(dado.ranges).round(decimals=2)

def bumpeou(dado):
	global bumper
	bumper = dado.data


# Código Principal
if __name__=="__main__":

	rospy.init_node("le_scan")
	topico_imagem = "/kamera"

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_bumper = rospy.Subscriber("/bumper", UInt8, bumpeou)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)


	while not rospy.is_shutdown():
		bumper = None

		velocidade = Twist(Vector3(0.0, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(velocidade)
		rospy.sleep(0.1)



# Laser
	# Frente
		if laser[0] < 0.2 and laser[0] != 0.0:
			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
			velocidade_saida.publish(velocidade)
			rospy.sleep(1.7)

	# Laterais Frontais
		if laser[30] < 0.2 and laser[30] != 0.0:
			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
			velocidade_saida.publish(velocidade)
			rospy.sleep(1.7)

		if laser[330] < 0.2 and laser[330] != 0.0:
			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
			velocidade_saida.publish(velocidade)
			rospy.sleep(1.7)

	# Laterais Traseiras
		if laser[160] < 0.2 and laser[160] != 0.0:
			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
			velocidade_saida.publish(velocidade)

		if laser[210] < 0.2 and laser[210] != 0.0:
			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
			velocidade_saida.publish(velocidade)
			rospy.sleep(1.7)

	# Trás
		if laser[180] < 0.2 and laser[180] != 0.0:
			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
			velocidade_saida.publish(velocidade)
			rospy.sleep(1.7)

# Bumper
		if bumper == 1:
			velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(3)

			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
			velocidade_saida.publish(velocidade)
			rospy.sleep(4)

		if bumper == 2:
			velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(3)

			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.5))
			velocidade_saida.publish(velocidade)
			rospy.sleep(4)

		if bumper == 3:
			velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(3)

			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
			velocidade_saida.publish(velocidade)
			rospy.sleep(4)

		if bumper == 4:
			velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(3)

			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.5))
			velocidade_saida.publish(velocidade)
			rospy.sleep(4)

		print('laser[0] = {}, laser[2] = {}, laser[357] = {}\nlaser[180] = {}, laser[182] = {}, laser[177] = {}\nbumper = {}\n' .format(laser[0], laser[2], laser[357], laser[180], laser[182], laser[177], bumper))