#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Esta classe deve conter todas as suas implementações relevantes para seu filtro de partículas
"""

from pf import Particle, create_particles
import numpy as np
import inspercles # necessário para o a função nb_lidar que simula o laser
import math
from scipy.stats import norm


largura = 775 # largura do mapa
altura = 748  # altura do mapa

# Robo
robot = Particle(largura/2, altura/2, math.pi/4, 1.0)

# Nuvem de particulas
particulas = []

num_particulas = 100


# Os angulos em que o robo simulado vai ter sensores
angles = np.linspace(0.0, 2*math.pi, num=8, endpoint=False)

# Lista mais longa
movimentos_longos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0],
			  [0,0,math.pi/12.0], [0, 0, math.pi/12.0], [0, 0, math.pi/12],[0,0,-math.pi/4],
			  [-5, 0, 0],[-5,0,0], [-5,0,0], [-10,0,0],[-10,0,0], [-10,0,0],[-10,0,0],[-10,0,0],[-15,0,0],
			  [0,0,-math.pi/4],[0, 10, 0], [0,10,0], [0, 10, 0], [0,10,0], [0,0,math.pi/8], [0,10,0], [0,10,0], 
			  [0,10,0], [0,10,0], [0,10,0],[0,10,0],
			  [0,0,-math.radians(90)],
			  [math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],
			  [math.cos(math.pi/3)*10, math.sin(math.pi/3),0]]

# Lista curta
movimentos_curtos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0]]

movimentos_relativos = [[0, -math.pi/3],[10, 0],[10, 0], [10, 0], [10, 0],[15, 0],[15, 0],[15, 0],[0, -math.pi/2],[10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [0, -math.pi/2], 
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [0, -math.pi/2], 
					   [10,0], [0, -math.pi/4], [10,0], [10,0], [10,0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0]]



movimentos = movimentos_relativos

def cria_particulas(minx=0, miny=0, maxx=largura, maxy=altura, n_particulas=num_particulas):
	for i in range(n_particulas):
		x = np.random.uniform(minx, maxx)
		y = np.random.uniform(miny, maxy)
		theta = np.random.uniform(0, 2 * math.pi)
		p = Particle(x, y, theta, w=1.0)
		particulas.append(p)
	return particulas
	
def move_particulas(particulas, movimento):
	for i in particulas:

		movei = i.move_relative(movimento)
		movelinx = norm.rvs(loc = i.__getitem__(0), scale = 1)
		moveliny = norm.rvs(loc = i.__getitem__(1), scale = 1)
		moveang = norm.rvs(loc = i.__getitem__(2), scale = 1/360 * 2 *math.pi)
		pose = [movelinx, moveliny, moveang]
		i.set_pose(pose)
	return particulas
	
def leituras_laser_evidencias(robot, particulas):

	leitura_robo = inspercles.nb_lidar(robot, angles)
	somatorio = 0

	for i in particulas:
		
	"""
		Realiza leituras simuladas do laser para o robo e as particulas
		Depois incorpora a evidência calculando
		P(H|D) para todas as particulas
		Lembre-se de que a formula $P(z_t | x_t) = \alpha \prod_{j}^M{e^{\frac{-(z_j - \hat{z_j})}{2\sigma^2}}}$ 
		responde somente P(D|Hi), em que H é a hi
		
		Esta função não precisa retornar nada, mas as partículas precisa ter o seu w recalculado. 
		
		Você vai precisar calcular para o robo
		
	"""
	
	
	
	# Voce vai precisar calcular a leitura para cada particula usando inspercles.nb_lidar e depois atualizar as probabilidades

def reamostrar(particulas, n_particulas = num_particulas):
	"""
		Reamostra as partículas devolvendo novas particulas sorteadas
		de acordo com a probabilidade e deslocadas de acordo com uma variação normal    
		
		O notebook como_sortear tem dicas que podem ser úteis
		
		Depois de reamostradas todas as partículas precisam novamente ser deixadas com probabilidade igual
		
		Use 1/n ou 1, não importa desde que seja a mesma
	"""
	return particulas