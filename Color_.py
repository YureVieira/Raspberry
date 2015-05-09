#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import cv
import numpy as np
import serial
import time
#----------------------#
img_height = 240
img_width = 320
#----------------------#
faixa_h = 5            #Tolerancia de matiz
faixa_s = 5            #Tolerancia de saturação
flag_Release = False   #Flag de liberação do mouse
ix,iy = -1,-1          #Posição do click
color = [101,0,0]              #Valor de hue
frame = np.zeros((img_height,img_width,3),np.uint8)
image_aux = np.zeros((img_height,img_width,3),np.uint8)
#----------------------------------------------------------------------------#
def desenhar(event,x,y,flags,param):
    global ix,iy,flag_Release,frame,color

    if event == cv2.EVENT_LBUTTONDOWN:
        flag_Release = True
        ix,iy = x,y

    elif event == cv2.EVENT_MOUSEMOVE:
        if flag_Release == True:
            ix,iy = x,y
            print ix,',',iy

    elif event == cv2.EVENT_LBUTTONUP:
        flag_Release = False
        _h,_s,_v = cv2.split(cv2.cvtColor(frame,cv.CV_BGR2HSV))
        color[0] = _h[iy][ix]
        color[1] = _s[iy][ix]
        color[2] = _v[iy][ix]
        print color[0]
        #cv2.imshow("tst",image_aux)

#----------------------------------------------------------------------------#
def nothing(x):
    pass
#----------------------------------------------------------------------------#
###########################################################################################
#-------------------------------------Classes---------------------------------------------#
###########################################################################################
class pid:
    #----------------------------------#
    kp = 0.0
    ki = 0.0
    kd = 0.0
    error = 0.0
    setpoint = 0.0
    input = 0.0
    output = 0.0
    Iterm = 0.0
    outMin = 0.0
    outMax = 0.0
    #----------------------------------#
    def set_limits(self, a, b):
        if a<b:
            self.outMin = a
            self.outMax = b
        else:
            self.outMin = b
            self.outMax = a
    #----------------------------------#
    def compute(self,_i):
        self.input = _i
        self.error = self.setpoint - self.input
        self.Iterm += (self.ki*self.error)
        if self.Iterm> self.outMax:
            self.Iterm= self.outMax
        elif self.Iterm < self.outMin:
            self.Iterm = self.outMin
        self.output = self.kp * self.error + self.Iterm

        if self.output > self.outMax :
            self.output = self.outMax
        elif self.output < self.outMin:
            self.output = self.outMin
        return self.output
    #----------------------------------#
###########################################################################################
#-------------------------------Rotina principal------------------------------------------#
###########################################################################################
capture = cv2.VideoCapture(0)
if capture.isOpened == False:
    capture.cv2.VideoCapture(1) 
capture.set(cv.CV_CAP_PROP_FRAME_HEIGHT,320)
capture.set(cv.CV_CAP_PROP_FRAME_WIDTH,240)
print capture
#Janelas
cv2.namedWindow('frame')
#cv2.namedWindow('Hue')
cv2.setMouseCallback('frame',desenhar)
center_screen = (int(capture.get(cv.CV_CAP_PROP_FRAME_WIDTH)/2),int(capture.get(cv.CV_CAP_PROP_FRAME_HEIGHT)/2));
#-----------------------------------------------------------------------------------------#
#Configurações da porta serial
# ser = serial.Serial(port="COM3",baudrate=9600)
ser = serial.Serial('/dev/ttyAMA0')
# ser.open()
ser.write(chr(202))
print ser.portstr
#-----------------------------------------------------------------------------------------#
var_erode = 5
#Controle PID
pid_x = pid()
pid_x.kp = 0.3
pid_x.ki = 0.03
pid_x.set_limits(0.0,180.0)
pid_x.setpoint = capture.get(cv.CV_CAP_PROP_FRAME_WIDTH)/2
pid_y = pid()
pid_y.kp = 0.3
pid_y.ki = 0.03
pid_y.set_limits(0.0,140.0)
pid_y.setpoint = capture.get(cv.CV_CAP_PROP_FRAME_HEIGHT)/2

print "Resolução",capture.get(cv.CV_CAP_PROP_FRAME_WIDTH),"X",capture.get(cv.CV_CAP_PROP_FRAME_HEIGHT)
print pid_x.setpoint
#-----------------------------------------------------------------------------------------#
#Loop
while True:
    ser.open()
    if capture.isOpened():
        rval,frame = capture.read()
    image_back = np.zeros(np.shape(frame),np.uint8) #Imagem onde será desenhado        
    #Dilatação
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(max(1,var_erode),max(1,var_erode)))
    #frame = cv2.erode(frame,kernel)
    #frame = cv2.dilate(frame,kernel)
    #--------------------------------------------------------------------------------------#
    #Separa os canais HSV
    im1,im2,im3 = cv2.split(cv2.cvtColor(frame,cv.CV_BGR2HSV))
    #--------------------------------------------------------------------------------------#
    #Limiar
    ret,im1 = cv2.threshold(im1,min(179,color[0]+faixa_h),255,cv.CV_THRESH_TOZERO_INV)
    ret,im1 = cv2.threshold(im1,max(0,color[0]-faixa_h),255,cv.CV_THRESH_TOZERO)
    ret,im1 = cv2.threshold(im1,1,255,cv.CV_THRESH_BINARY)

    # ret,im2 = cv2.threshold(im2,min(255,color[1]+faixa_s),255,cv.CV_THRESH_TOZERO_INV)
    # ret,im2 = cv2.threshold(im2,max(0,color[1]-faixa_s),255,cv.CV_THRESH_TOZERO)
    # ret,im2 = cv2.threshold(im2,1,255,cv.CV_THRESH_BINARY)
    #cv2.imshow('Hue', im1)
    # cv2.imshow('Sat', im2)
    #ret,result = cv2.threshold(im1,1,255,cv.CV_THRESH_BINARY)
    #--------------------------------------------------------------------------------------#
    #Busca contornos
    contours, hierarchy  = cv2.findContours(im1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #contours, hierarchy  = cv2.findContours(im1,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    #Obter o o indice do contorno de maior área
    num = 0
    blob_detect= False
    indice_max = 0
    area_min = 0
    area_blob=300
    num_contours = np.shape(contours)
    if num_contours[0] > 0:
        while num < num_contours[0]:
            M = cv2.moments(contours[num])
            if num > 0 & 1 :
                if M['m00'] > area_blob:
                    area_blob = M['m00']
                    indice_max = num
                    blob_detect = True
            num +=1
        if blob_detect == True:         #Se um blob com area relevante for detectado...
            cnt = contours[indice_max]
            #texto2 = 'N de blobs '+ str(num_contours[0])+', blob de maior area e o ' + str(indice_max)+'(Área = ' + str(area_max) + ')'
            #print area_blob        
            #obtenha os momentos
            M = cv2.moments(cnt)
            (Cx,Cy),radius = cv2.minEnclosingCircle(cnt)
            cx = int(Cx)
            cy = int(Cy)
            raio = int(radius)
             
            #cv2.drawContours(frame,contours,1,(0,0,255),-1)
            #desenhe os contornos na imagem branca o segundo contorno
            #cv2.drawContours(fundo,contours,1,255,-1)
            cv2.drawContours(image_back,contours,indice_max,(255,255,255),-2)
            cv2.circle(frame,(cx,cy),raio,(0,255,255),3)
            #print texto2
            out_x = pid_x.compute(cx)
            out_y = pid_y.compute(cy)
            # print "setpoint: ",pid_x.setpoint, " / Input: ",pid_x.input," / Erro: ",pid_x.error," / Saida: ",out
            # data_x = 'a'
            # data_x = chr(int(out))
            # data_y = 'a'
            # data_y = chr(int(out))
            print int(out_x),',',int(out_y)
            #Escrita na Porta Serial
            ser.write(chr(200))
            ser.write(chr(int(out_x)))
            ser.write(chr(201))
            ser.write(chr(int(out_y)))

            if cx > center_screen[0]:
                cv2.line(frame,center_screen,(cx,cy),(0,255,0),2);
            else:
                cv2.line(frame,center_screen,(cx,cy),(0,0,255),2);
    cv2.circle(frame,(ix,iy),12,(0,255,0),3)
    #-------------------------------------------------------------------------------------#
    #Desenhos
    cv2.circle(frame,center_screen,5,(10,255,50),3,2);        #Circulo qua marca o centro.
    #-------------------------------------------------------------------------------------#
    #cv2.imshow('back', image_back)
    cv2.imshow('frame', frame)
    key = cv2.waitKey(10)
    if(key == ord('q')):
        break
#Fim do loop
    try:
        ser.close()
    except:
        None
cv2.destroyAllWindows()
#exit(0)

