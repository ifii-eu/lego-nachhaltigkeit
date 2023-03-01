import cv2 
import numpy as np
import copy
import heapq
import math

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.g = 0
        self.h = 0

    def __lt__(self, other):
        return self.g + self.h < other.g + other.h

class AStar:
    def __init__(self, grid):
        self.grid = grid
        self.start = None
        self.end = None
        self.nodes = []

    def get_neighbors(self, node):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                x = node.x + dx
                y = node.y + dy
                if x < 0 or y < 0 or x >= len(self.grid) or y >= len(self.grid[0]):
                    continue
                if self.grid[x][y] == 1:
                    continue
                neighbors.append((x, y))
        return neighbors

    def get_distance(self, node1, node2):
        dx = abs(node1.x - node2.x)
        dy = abs(node1.y - node2.y)
        return math.sqrt(dx*dx + dy*dy)

    def search(self, start, end):
        self.start = Node(start[0], start[1])
        self.end = Node(end[0], end[1])
        open_list = []
        closed_list = set()

        heapq.heappush(open_list, self.start)

        while open_list:
            current_node = heapq.heappop(open_list)

            if current_node == self.end:
                path = []
                while current_node:
                    path.append((current_node.x, current_node.y))
                    current_node = current_node.parent
                path.reverse()
                return path

            closed_list.add((current_node.x, current_node.y))

            for neighbor in self.get_neighbors(current_node):
                neighbor_node = Node(neighbor[0], neighbor[1], current_node)

                if (neighbor_node.x, neighbor_node.y) in closed_list:
                    continue

                neighbor_node.g = current_node.g + self.get_distance(current_node, neighbor_node)
                neighbor_node.h = self.get_distance(neighbor_node, self.end)

                if neighbor_node in open_list:
                    if neighbor_node.g > current_node.g:
                        continue

                heapq.heappush(open_list, neighbor_node)
                self.nodes.append(neighbor_node)

        return None

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

#for x in range(6):
#    for y in range(4):
#        display[x+2][y+4]="grey"


display[2][0]="red"
display[6][6]="red"
display[7][6]="red"
display[6][5]="red"
display[7][5]="red"


display[1][0]="red"
display[2][0]="red"

for x in range(platex):
    for y in range(platey):
        block[x][y]=display[x][y]        



def check(color, color_name):
    checked=copy.deepcopy(color)
    for x in range(len(color)):
        for y in range(len(color[x])):
            if(checked[x][y]!=0 and checked[x][y]!="grey" and checked[x][y]==color_name):
                s=0
                if(checked[x][y]==checked[x][y+1] and checked[x][y]==checked[x+1][y] and checked[x][y]==checked[x+1][y+1]):
                    if(checked[x-1][y]==0):
                        start=(x-1,y)
                    if(checked[x-1][y+1]==0):   
                        start=(x-1,y+1)
                    if(checked[x+2][y]==0):
                        start=(x+2,y)
                    if(checked[x+2][y+1]==0):   
                        start=(x+2,y+1)
                    if(checked[x][y-1]==0):
                        start=(x,y-1)
                    if(checked[x+1][y-1]==0):
                        start=(x+1,y-1)
                    if(checked[x][y+2]==0):
                        start=(x,y+2)
                    if(checked[x+1][y+2]==0):
                        start=(x+1,y+2)

                if(checked[x][y]==checked[x][y+1] and s!=1):

                    end1=(x,y)
                    #end2=(x,y+1)
                    checked[x][y]=0
                    checked[x][y+1]=0

                elif(checked[x][y]==checked[x+1][y] and s!=1):
                    end1=(x,y)
                    #end2=(x+1,y)
    return start,end1
    
    

    return color
###Als letztes 

# def checkcolorout(color):
#     checked=copy.deepcopy(color)

#     for x in range(len(color)):
#         for y in range(len(color[x])):
#             if(checked[x][y]!=0 and checked[x][y]!="grey"):
#                 if(checked[x][y]==checked[x][y+1]):
#                     color[x][y]+="_out"
#                     color[x][y+1]+="_out"
#                     checked[x][y]=0
#                     checked[x][y+1]=0

#                 elif(checked[x][y]==checked[x+1][y]):
#                     color[x][y]+="_out"
#                     color[x+1][y]+="_out"
#                     checked[x+1][y]=0
#                     checked[x][y]=0
#                 checked[x][y]=0
#     return color



# for x in range(platex):
#     for y in range(platey):
#         cluster=[]
#         if(block[x][y]!=0):
#             minx=platex
#             maxx=0
#             miny=platey
#             maxy=0
#             cluster=checkcluster(x,y,cluster, block)
#             diffx=(maxx-minx)+1
#             diffy=(maxy-miny)+1
#             machinepos.append((x-1,y-1))


#             globals()['machines%s' % n]=[[0]*(diffy+2) for i in range(diffx+2)]
#             for i in range(len(cluster)):

#                 if(cluster[i][0]==0):
#                     globals()['machines%s' % n][cluster[i][0]-minx][cluster[i][1]+1-miny]="b"
#                 elif(cluster[i][1]==0):
#                     globals()['machines%s' % n][cluster[i][0]+1-minx][cluster[i][1]-miny]="b"
#                 elif(cluster[i][0]==platex):
#                     globals()['machines%s' % n][cluster[i][0]+2-minx][cluster[i][1]+1-miny]="b"
#                 elif(cluster[i][1]==platey):
#                     globals()['machines%s' % n][cluster[i][0]+1-minx][cluster[i][1]+2-miny]="b"
                
#                 globals()['machines%s' % n][cluster[i][0]+1-minx][cluster[i][1]+1-miny]=display[cluster[i][0]][cluster[i][1]]  
#                 #print(globals()['machines%s' % n], n)
#                 print(checkcolorout(globals()['machines%s' % n], n))
#             n+=1
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


# for ncount in range(0,len(machinepos)):
#     for x in range(len(globals()['machines%s' % ncount])):
#          for y in range(len(globals()['machines%s' % ncount][0])):

#             xcor=machinepos[ncount][0]+x
#             ycor=machinepos[ncount][1]+y
#             board[xcor][ycor]=globals()['machines%s' % ncount][x][y]


    
                 
print(display)
start, end=check(display,"red")
print (start)
astar = AStar(display)
path = astar.search(start, end)

if path:
    distance = len(path)
    print("Gesamtstrecke:", distance)
else:
    print("Kein Pfad gefunden")

x, y = pixel, pixel
h, w = lego_orange.shape[:2]

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


