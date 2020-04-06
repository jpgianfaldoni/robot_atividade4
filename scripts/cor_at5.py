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
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from cv_bridge import CvBridge, CvBridgeError
import cormodule
from sensor_msgs.msg import LaserScan

dados = []

bridge = CvBridge()

cv_image = None
media = []
id = -1
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

colorLowRange = (36, 80, 80)
colorHighRange = (70, 255,255)
frame_hsv = None

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	#print("frame")
	global cv_image
	global media
	global centro
	global maior_area
	global colorLowRange
	global colorHighRange
	global frame_hsv


	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	#print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		# cv_image = cv2.flip(cv_image, -1)
		frame_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		

		media, centro, maior_area =  cormodule.identifica_cor(cv_image, colorLowRange, colorHighRange) 
		#verde (36,80, 80), (70,255,255)
		
		
		
		
		depois = time.clock()
		# cv2.imshow("Camera", cv_image)
		# print("Média: ", media)
		# print("Centro: ", centro)
		# print("Maior Área: ", maior_area)

	except CvBridgeError as e:
		#print('ex', e)
		print("")

def recebe(msg):
	#global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
	#global y
	#global z
	global id
	for marker in msg.markers:
		id = marker.id
		"""
		marcador = "ar_marker_" + str(id)

		print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
		header = Header(frame_id=marcador)
		# Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
		# Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
		# Nao ser que queira levar angulos em conta
		trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
		
		# Separa as translacoes das rotacoes
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
		# Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
		# no eixo X do robo (que e'  a direcao para a frente)
		t = transformations.translation_matrix([x, y, z])
		# Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
		r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
		m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
		z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
		v2 = numpy.dot(m, z_marker)
		v2_n = v2[0:-1] # Descartamos a ultima posicao
		n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
		x_robo = [1,0,0]
		cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
		angulo_marcador_robo = math.degrees(math.acos(cosa))

		# Terminamos
		print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))"""

		print(id)

def scaneou(dado):
	global dados
	dados = dado.ranges

def findCenter():
	while True:
		direction = media[0] - centro[0]
		if direction >= 1: 
			vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
		elif direction < 1:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
		velocidade_saida.publish(vel)
		if abs(direction) < 10:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			return

def moveForward():
	while True:
		direction = media[0] - centro[0]
		lista1 = [x<1 for x in dados]
		if direction >= 1: 
			vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.1))
		elif direction < 1:
			vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.1))
		velocidade_saida.publish(vel)
		if np.any(lista1) and id == 3:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			return True
		elif np.any(lista1):
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			reset()
			return False

def reset():
	#3.5s
	vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
	velocidade_saida.publish(vel)
	rospy.sleep(3.5)


def leTouch():
	global frame_hsv
	global colorHighRange
	global colorLowRange
	global dados

	approach = True
	while approach:
		corCentro = cv2.inRange(frame_hsv, colorLowRange, colorHighRange) 
		if corCentro[centro[0]][centro[1]]:
			if dados[0] > 0.2 and dados[0] < 1:
				vel = Twist(Vector3(0.01,0,0), Vector3(0,0,0))
				velocidade_saida.publish(vel)
			elif dados[0] > 0.5:
				return False
			else:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				velocidade_saida.publish(vel)
				return True	
		else:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			return False


if __name__=="__main__":
	rospy.init_node("cor")

	topico_imagem = "/camera/rgb/image_raw/compressed"

	recebe_mark = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos
	
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
					inFront = moveForward()
					inFront = leTouch()
				else:
					vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.3))
					velocidade_saida.publish(vel)

			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
