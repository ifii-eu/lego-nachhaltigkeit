import cv2 
import numpy as np
import sys


class machiene(object): 
    def __init__(self, inhaber, kontonummer, kontostand, kontokorrent=0): 
        self.Inhaber = inhaber 
        self.Kontonummer = kontonummer 
        self.Kontostand = kontostand 
        self.Kontokorrent = kontokorrent

#Variables
pixel=12
plate=36



window=True
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW) #video aufzeichhnung 
display = [[0]*plate for i in range(plate)]
drange = [[0]*plate for i in range(plate)]



#definierung
for x in range(6):
    for y in range(4):
        display[x+2][y+4]="grey"
display[6][5]="red"
display[6][6]="red"
display[7][5]="red"
display[7][6]="red"
display[9][9]="orange"
display[10][9]="orange"
display[9][10]="orange"
display[10][10]="orange"
display[1][1]="pink"



block=display



for x in range(plate):
        for y in range(plate):
            if(block[x][y]!=0):

                block[x][y]=0
                if(block[x+1][y]!=0):

                block[x][y]

                




lego_orange=cv2.imread("Lego/Lego_Orange.png")
lego_orange=cv2.resize(lego_orange,(pixel,pixel))

lego_white=cv2.imread("Lego/Lego_White.png")
lego_white=cv2.resize(lego_white,(pixel,pixel))

lego_black=cv2.imread("Lego/Lego_Black.png")
lego_black=cv2.resize(lego_black,(pixel,pixel))

lego_pink=cv2.imread("Lego/Lego_Pink.png")
lego_pink=cv2.resize(lego_pink,(pixel,pixel))

lego_red=cv2.imread("Lego/Lego_Red.png")
lego_red=cv2.resize(lego_red,(pixel,pixel))


x, y = pixel, pixel
h, w = lego_orange.shape[:2]



while window:
    ret, img = cap.read()

    for x in range(plate):
        for y in range(plate):
            xx=x*pixel
            yy=y*pixel
            if(display[x][y]==0):
                img[yy:yy+h, xx:xx+w] = lego_white
            elif(display[x][y]=="orange"):
                img[yy:yy+h, xx:xx+w] = lego_orange
            elif(display[x][y]=="grey"):
                img[yy:yy+h, xx:xx+w] = lego_black
            elif(display[x][y]=="red"):
                img[yy:yy+h, xx:xx+w] = lego_red
            elif(display[x][y]=="pink"):
                img[yy:yy+h, xx:xx+w] = lego_pink


    cv2.namedWindow("img", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("img", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("img", img)
    #cv2.namedWindow("ret", cv2.WND_PROP_FULLSCREEN)
    #cv2.setWindowProperty("ret", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    #cv2.imshow("ret", ret)

    res=cv2.waitKey(10)
    if res == ord('q')or ord('Q'):
        window=False


