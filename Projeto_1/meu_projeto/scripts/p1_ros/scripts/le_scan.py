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

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9
viu_dog = False
viu_gato = False

area = 0.0
check_delay = False


# Funções de Leitura
bumper = None
laser = [10]*360
def scaneou(dado):
	global laser
	laser = np.array(dado.ranges).round(decimals=2)

def bumpeou(dado):
	global bumper
	bumper = dado.data

def roda_todo_frame(imagem):
	global cv_image
	global media
	global centro
	global viu_dog
	global viu_gato

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		return 
	antes = time.clock()
	cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
	centro, imagem, resultados =  visao_module.processa(cv_image)

	for r in resultados:
		if r[0] == "dog":
			viu_dog = True
			print("cell phhone\n\n\n\n\n\n")

		elif r[0] == "cat":
			viu_gato = True
			print("gato\n\n\n\n\n\n")
		depois = time.clock()

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

# Identificador de Objetos
		if viu_dog:
			velocidade = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(0.8)
			viu_dog = False

		if viu_gato:
			velocidade = Twist(Vector3(-0.1,0,0), Vector3(0,0,0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(0.8)
			viu_gato = False

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
