#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule
from sensor_msgs.msg import LaserScan

dados = []

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global maior_area

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		cv_image = cv2.flip(cv_image, -1)

		

		media, centro, maior_area =  cormodule.identifica_cor(cv_image,(36, 80, 80), (70, 255,255)) 
		#verde (36,80, 80), (70,255,255)
		
		
		
		
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
		# print("Média: ", media)
		# print("Centro: ", centro)
		# print("Maior Área: ", maior_area)

	except CvBridgeError as e:
		#print('ex', e)
		print("")

def scaneou(dado):
	global dados
	dados = dado.ranges

def findCenter():
	while True:
		direction = media[0] - centro[0]
		if direction > 1: 
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
		elif direction < 1:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
		velocidade_saida.publish(vel)
		if abs(direction) < 10:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			return

def moveForward(inFront):
	while True:
		direction = media[0] - centro[0]
		if direction > 1: 
			vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.1))
		elif direction < 1:
			vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.1))
		velocidade_saida.publish(vel)
		if dados[0] < 0.30:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			inFront = True
			return inFront

def leTouch():
	vel = Twist(Vector3(0.01,0,0), Vector3(0,0,0))
	velocidade_saida.publish(vel)
	while True:
		print("leTouch")
		time.sleep(1)

if __name__=="__main__":
	rospy.init_node("cor")

	topico_imagem = "/kamera"
	
	# Para renomear a *webcam*
	#   Primeiro instale o suporte https://github.com/Insper/robot19/blob/master/guides/debugar_sem_robo_opencv_melodic.md
	#
	#	Depois faça:
	#	
	#	rosrun cv_camera cv_camera_node
	#
	# 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
	#
	# 
	# Para renomear a câmera simulada do Gazebo
	# 
	# 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
	# 
	# Para renomear a câmera da Raspberry
	# 
	# 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
	# 

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	inFront = False

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if len(media) != 0 and len(centro) != 0 and inFront == False:
				if maior_area>1000:
					findCenter()
					inFront = moveForward(inFront)
					leTouch()
				else:
					vel = Twist(Vector3(0,0,0), Vector3(0,0,0.3))
					velocidade_saida.publish(vel)

			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
