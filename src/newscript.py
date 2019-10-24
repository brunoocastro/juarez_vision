#!/usr/bin/env python
# -*- coding: utf-8 -*-

#importing the packages
#from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
import threading
import rospy

from op3_walking_module_msgs.msg import WalkingParam

#import init

# Define a variavel onde vamos alterar os parametros
global move
move = WalkingParam()

# Time Waiting - Variavel para ajustar o tempo de espera
# entre o primeiro comando de giro e o proximo
tw = 0.1

# Velocidades maxima e minima de giro do robo
vel_max = 0.005
vel_min = -0.005

#Altura na tela onde comeca a percorrer
htela = 130
htelab = htela + 40

# Responsavel pela movimentacao para a esquerda
# Ainda nao foi feita a logica

def calc_vel(perc):   
    delta = vel_max - vel_min

    result = ((delta*perc)/100)

    return result


def turn_left(perc):
    global move
    print('[Movendo para a esquerda com velocidade de {}%]'.format(perc))
    print('Setando - Velocidade de giro = {}'.format(calc_vel(perc)))
    move.angle_move_amplitude = calc_vel(perc)
    time.sleep(tw)

#Responsavel pela movimentacao para a direita
#Ainda nao foi feita a logica
def turn_right(perc):
    global move
    print('[Movendo para a direita com velocidade de {}%]'.format(perc))
    print('Setando - Velocidade de giro = {}'.format(calc_vel(perc)))
    move.angle_move_amplitude = calc_vel(perc)
   
    time.sleep(tw)

def move_foward():
    global move
    move.x_move_amplitude = 0.008

def stop():
    global move
    move.x_move_amplitude = 0.0
    move.y_move_amplitude = 0.0
    move.z_move_amplitude = 0.0

def talker():
    global move
    thread1 = threading.Thread(target=vision())
    thread1.start()
    pub = rospy.Publisher('robotis/walking/set_params', WalkingParam, queue_size=10)
    rospy.on_shutdown(stop)
    rospy.init_node('vision', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(move)
        rate.sleep()
        
def vision():
    vs = VideoStream(src=0).start()

    time.sleep(1.0)

    # keep looping
    while not rospy.is_shutdown():

        # allow the camera or video file to warm up
        time.sleep(0.1)
	
	# grab the current frame
        frame = vs.read()
        #Resize da imagem, transformando ela para (300px x 300px)
        img = cv2.resize(frame,(300, 300))
        #Criando uma outra imagem de 10x300 para diminuir o tempo de processamento
        img2 = img[htela:htelab,0:300]

        #Definindo os parametos para identificacao do branco (Cor em HSV)
        
        low = (0,0,0)
        up = (200,200,90)

        #Branco
        #low = (0,0,200)
        #up = (150,150,255)

        #Convertendo de BGR para HSV
        hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

        # Aplica a mascara
        # Na imagem, ele localiza os pontos que estão entre os ranges definidos de cor
        # Ou seja, quando a cor de um pixel for maior que o valor minimo definido (low)
        # e menor que o maximo (up) ele transforma essa posicao da imagem em um ponto branco
        mask = cv2.inRange(hsv, low, up)
        # O erode reduz o pixel em 1, pra remover os ruidos, se for apenas um pixel perdido ele ignora.
        #mask = cv2.erode(mask, None, iterations=1)
        # O dilate expande os pixels que não foram excluidos, formando uma regiao marcada
        mask = cv2.dilate(mask, None, iterations=6)

        #Encontra o centro da area e da o output nele
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)

        cnts = imutils.grab_contours(cnts)
        pos = []

        #cria o vetor com todas as posições encontradas nos centros das areas brancas
        for c in cnts:
            M = cv2.moments(c)
            if (M["m00"] != 0):
                p = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                pos.append(p)

        cv2.rectangle(img,(0,htela),(300,htelab),(0,255,0),2)

        #Se existir mais de um ponto no vetor, ou seja, diferente de zero o numero de itens
        if (len(pos)!=0):

            # Calcula e define os pontos extremos direito e esquerdo identificados na imagem
            #pe = pos[(len(pos)-1)][0]
            #pe = pos[0][0]
            pd = np.amax(pos, axis=0)[0]
            #pd = pos[0][0]
            pe = np.amin(pos, axis=0)[0]
            #Ponto direito a partir da extrema direita da imagem
            cpd = 300-pd

            # Define os ranges de distância minima, em pixels, na qual o robô nao vai atuar
            # Explicando melhor, se a distancia da direita e a da esquerda forem maior de 60px
            # Continua andando normal, se for menor, ai sim corrige a tragetória.
            diste = 60
            distd = 60

            print('\nNumero de pontos: ' + str(len(pos)))
            print('Pontos: ' + str(pos))
            print('PE (Azul)     = [{}]\nPD (Vermelho) = [{}]\nPEF = [{}]\n'.format(pe,pd,cpd))

            #Desenha na tela quais pontos ele ta identificando
            cv2.rectangle(img,(0,htela),(300,htelab),(0,255,0),2)
            cv2.circle(img, (pe, htelab - 10), 5, (255,0,0),2)
            cv2.circle(img, (pd, htelab - 10), 4, (0,0,255),2)

            #Se houver apenas 1 linha na pista
            #Se houver apenas 1 linha na pista
            if len(pos) == 1 :
                # Pegando a linha da Direita, ou seja, distância maior que a metade da tela:
                if pos[0][0] > 150:
                    turn_left(60)

                # Pegando a linha da Esquerda, ou seja, distância menor que a metade da tela:
                if pos[0][0] < 150:
                    turn_right(60)


            # Identificando mais de 1 linha na pista
            if len(pos)>1:

                # Se a distância da direita for menor que 60px a partir a extrema direita
                # Ele chama a função responsável por virar para a esquerda, passando como 
                # parâmetro a porcentagem da velocidade max que queremos que seja aplicada.
                if cpd < diste:
                    turn_left(40)

                # Se a distância da direita for menor que 60px a partir a extrema direita
                # Ele chama a função responsável por virar para a esquerda, passando como 
                # parâmetro a porcentagem da velocidade max que queremos que seja aplicada.
                elif pe < distd:
                    turn_right(40)

                else:
                    print("Continuar andando")
                    move.angle_move_amplitude = 0
                    move_foward()

        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            break

        # show the frame to our screen
        #cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("IMG", img)

        key = cv2.waitKey(1) & 0xFF


        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break


    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
