import cv2 
import numpy as np
import copy
#import sys

#Variables
platex=36
platey=34
pixel=27
xachse=500
yachse=50


#Variables

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



#Variables


window=True
back = cv2.imread("backgroundjuice.png")

machinepos=[]
display = [[0]*platey for i in range(platex)]
machine = [[0]*platey for i in range(platex)]
board= [[0]*platey for i in range(platex)]
machines=[]
block = machine.copy()
n=0




#definierung
# for x in range(6):
#     for y in range(4):
#         display[x+10][y+10]="orange"

for x in range(6):
    for y in range(4):
        display[x+2][y+4]="grey"



display[1][0]="red"
display[2][0]="red"

for x in range(platex):
    for y in range(platey):
        block[x][y]=display[x][y]        

def checkcluster(x, y, cluster, block):
    if block[x][y] == 0:
        return cluster
    cluster.append((x, y))
    block[x][y] = 0
    global minx, maxx, miny, maxy
    minx = min(minx, x)
    maxx = max(maxx, x)
    miny = min(miny, y)
    maxy = max(maxy, y)
    for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
        if 0 <= x+dx < len(block) and 0 <= y+dy < len(block[0]):
            cluster = checkcluster(x+dx, y+dy, cluster, block)
    return cluster

def checkcolorin(color, nr):
    checked=copy.deepcopy(color)

    for x in range(1,len(color)-1):
        for y in range(1,len(color[x])-1):
            if(checked[x][y]!=0 and checked[x][y]!="grey"):
                if(checked[x][y]==checked[x][y+1] and checked[x][y]==checked[x+1][y] and checked[x][y]==checked[x+1][y+1]):
                    print("4")
                    if(checked[x-1][y]==0):
                        color[x-1][y]=color[x][y]+"_in" 
                    if(checked[x-1][y+1]==0):   
                        color[x-1][y+1]=color[x][y]+"_in" 
                    if(checked[x+2][y]==0):
                        color[x+2][y]=color[x][y]+"_in" 
                    if(checked[x+2][y+1]==0):   
                        color[x+2][y+1]=color[x][y]+"_in" 
                    if(checked[x][y-1]==0):
                        color[x][y-1]=color[x][y]+"_in" 
                    if(checked[x+1][y-1]==0):
                        color[x+1][y-1]=color[x][y]+"_in" 
                    if(checked[x][y+2]==0):
                        color[x][y+2]=color[x][y]+"_in" 
                    if(checked[x+1][y+2]==0):
                        color[x+1][y+2]=color[x][y]+"_in"

                    checked[x][y+1]=0
                    checked[x+1][y]=0
                    checked[x+1][y+1]=0
###Als letztes 

def checkcolorout(color, nr):
    checked=copy.deepcopy(color)

    for x in range(1,len(color)-1):
        for y in range(1,len(color[x])-1):
            if(checked[x][y]!=0 and checked[x][y]!="grey"):
                if(checked[x][y]==checked[x][y+1]):
                    if(checked[x-1][y]==0):
                        color[x-1][y]=color[x][y]+"_in" 
                    if(checked[x+1][y]==0):   
                        color[x+1][y]=color[x][y]+"_in"
                    if(checked[x][y-1]==0):
                        color[x][y-1]=color[x][y]+"_in"
                    if(checked[x][y+2]==0):
                        color[x][y+2]=color[x][y]+"_in"
                        

                    if(checked[x+1][y+1]==0):
                        color[x+1][y+1]=color[x][y]+"_in"
                    if(checked[x-1][y+1]==0):
                        color[x-1][y+1]=color[x][y]+"_in"

                    checked[x][y]=0
                    checked[x][y+1]=0

                elif(checked[x][y]==checked[x+1][y]):
                    if(checked[x][y-1]==0):
                        color[x][y-1]=color[x][y]+"_in" 
                    if(checked[x][y+1]==0):   
                        color[x][y+1]=color[x][y]+"_in"
                    if(checked[x-1][y]==0):
                        color[x-1][y]=color[x][y]+"_in"
                    if(checked[x+2][y]==0):
                        color[x+2][y]=color[x][y]+"_in"

                    if(checked[x+1][y+1]==0):
                        color[x+1][y+1]=color[x][y]+"_in"
                    if(checked[x+1][y-1]==0):
                        color[x+1][y-1]=color[x][y]+"_in"

                    checked[x+1][y]=0
                    checked[x][y]=0
                checked[x][y]=0
    return color



for x in range(platex):
    for y in range(platey):
        cluster=[]
        if(block[x][y]!=0):
            minx=platex
            maxx=0
            miny=platey
            maxy=0
            cluster=checkcluster(x,y,cluster, block)
            diffx=(maxx-minx)+1
            diffy=(maxy-miny)+1
            machinepos.append((x-1,y-1))


            globals()['machines%s' % n]=[[0]*(diffy+2) for i in range(diffx+2)]
            for i in range(len(cluster)):

                if(cluster[i][0]==0):
                    globals()['machines%s' % n][cluster[i][0]-minx][cluster[i][1]+1-miny]="b"
                elif(cluster[i][1]==0):
                    globals()['machines%s' % n][cluster[i][0]+1-minx][cluster[i][1]-miny]="b"
                elif(cluster[i][0]==platex):
                    globals()['machines%s' % n][cluster[i][0]+2-minx][cluster[i][1]+1-miny]="b"
                elif(cluster[i][1]==platey):
                    globals()['machines%s' % n][cluster[i][0]+1-minx][cluster[i][1]+2-miny]="b"
                
                globals()['machines%s' % n][cluster[i][0]+1-minx][cluster[i][1]+1-miny]=display[cluster[i][0]][cluster[i][1]]  
                print(globals()['machines%s' % n], n)
                checkcolorout(globals()['machines%s' % n], n)
            n+=1
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


for ncount in range(0,len(machinepos)):
    for x in range(len(globals()['machines%s' % ncount])):
         for y in range(len(globals()['machines%s' % ncount][0])):

            xcor=machinepos[ncount][0]+x
            ycor=machinepos[ncount][1]+y
            board[xcor][ycor]=globals()['machines%s' % ncount][x][y]


    
            
        


x, y = pixel, pixel
h, w = lego_orange.shape[:2]

display=board
while window:
    img=back
    for x in range(platex):
        for y in range(platey):
            xx=x*pixel+xachse
            yy=y*pixel+yachse
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
    #cv2.setWindowProperty("img", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("img", img)
    #cv2.namedWindow("ret", cv2.WND_PROP_FULLSCREEN)
    #cv2.setWindowProperty("ret", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    #cv2.imshow("ret", ret)
    res=cv2.waitKey(10)
    if res == ord('q'):
        window=False


