import cv2
from object_detector import*
import matplotlib.pyplot as plt
import numpy as np


#cv2.namedWindow("output", cv2.WINDOW_NORMAL)
img1 = cv2.imread('/home/raspi/Desktop/imageFilament.jpg')
img = cv2.resize(img1, (1800,1080))  # resolution de la photo 


imgBegin = img[0:960, 430:530]  # on rogne pour economiser les ressources 

# filtre RGB ?
img = cv2.cvtColor(imgBegin, cv2.COLOR_BGR2RGB)      

# on separe les couches rouge vert bleu en niveau de gris 
red, green, blue  = cv2.split(img)    

# calibrer le seuil de detection, plus il est bas plus la photo du fil est gros
seuil = 120 

# on utilise la couche rouge en niveau de gris et on fixe un seuil chaque pixel devient noir si inferieur au seuil, sinon blanc 
ret,seg_red = cv2.threshold(red,seuil,255,cv2.THRESH_BINARY_INV)          

# show
cv2.imshow('cut',imgBegin)
cv2.imshow('RGB',img)
cv2.imshow('grey',red)
cv2.imshow('B/W',seg_red)


cv2.waitKey(0)


# draw contours / get high , bright ?

#contour, hierarchy = cv2.findContours(seg_red,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
#cv2.drawContours(seg_red, [cnt], -1 , (0,255,0),3)
#cv2.imshow('seg_red',seg_red)









