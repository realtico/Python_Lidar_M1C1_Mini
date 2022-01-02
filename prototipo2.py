# -*- coding: utf-8 -*-
"""
Created on Sun Jul 11 13:26:17 2021

@author: hlpp
"""


import serial
from serial.tools.list_ports import comports
from math import atan
import pygame
from math import pi, sin, cos

coord_diff=lambda x,y:(x[0]-y[0],x[1]-y[1])
size_scale=lambda x,s:(x[0]/s,x[1]/s)
             
def polar_to_point(polar,center=(320,320), scale=40.0, calibration=1.4):
    ang=polar[0]*pi/180.0
    #print (polar[0])
    dist=polar[1]/scale*calibration
#    return (center[0]+dist*sin(ang),center[1]-dist*cos(ang)) #posição original
    return (center[0]-dist*sin(ang),center[1]+dist*cos(ang)) #posição natural (180 de rotação)
            
class Lidar ():
    def __init__(self, port):
        try:
            self.sport=serial.Serial(port,115200,timeout=1)
            self.__initOK__=True
            print ("pressione ESC para sair")
        except:
            print (f"Could not open {port}")
            self.__initOK__=False
        self.gridstyle=0
        self.backcolor=(255,255,255)
        self.gridcolor=(0,0,128)
        self.lessergridcolor=(192,192,255)
        self.gridcentercolor=(0,0,0)
        self.__newpack__=[]
        self.DataPack=self.__newpack__.copy()
        self.scale=40
        self.__init_screen__()
        self.__put_grid__(self.gridstyle)
        pygame.display.update()
        
    def __init_screen__(self):
        pygame.init()
        self.screen=pygame.display.set_mode((640,640))
        pygame.font.init()
        pygame.display.set_caption('Lidar data')
        self.screen.fill(self.backcolor)
        self.myfont=pygame.font.SysFont('Calibri',16)
        self.Hugefont=pygame.font.SysFont('Calibri',60,bold=True)
    def __put_grid__(self, style=0):
        if style==0: #polar (default)
            for i in range(8):
                pygame.draw.circle(self.screen,self.gridcolor,(320,320),(i+1)*40,width=1)
                for j in range(8,40,8):
                    pygame.draw.circle(self.screen,self.lessergridcolor,(320,320),(i+1)*40-j,width=1)
            for i in range(0,360,90):
                for j in range (15,90,15):
                    pygame.draw.line(self.screen,self.lessergridcolor,(320,320),(320+320.0*sin(pi*(i+j)/180.0),320+320.0*cos(pi*(i+j)/180.0)),1)           
        else: #rectangular
            for i in range(0,641,8):
                color=self.lessergridcolor
                pygame.draw.line(self.screen, color, (0,i),(640, i))                
                pygame.draw.line(self.screen, color, (i,0),(i,640))                
            for i in range(0,641,40):
                color=self.gridcolor
                pygame.draw.line(self.screen, color, (0,i),(640, i))                
                pygame.draw.line(self.screen, color, (i,0),(i,640))                
        i=320
        color=self.gridcentercolor
        pygame.draw.line(self.screen, color, (0,i),(640, i))                
        pygame.draw.line(self.screen, color, (i,0),(i,640))                

        for i in range(8):
            dist=(i+1)*1000*(self.scale/40)
            text=self.myfont.render(f"{dist} mm",False,self.gridcolor)
            self.screen.blit(text,(330,320-((i+1)*40)))
            self.screen.blit(text,(330,320+((i+1)*40)))
            
                             
    def run(self):
        global flag_lidar_running
        global flag_changed
        global CurrentFrame
        
        if not self.__initOK__:
            print ('Fail to Initialize')
            return
        
        self.running=True 
        flag_lidar_running=True
        while self.running:
            try:
                c=self.sport.read()
            except:
                c=0x00
            if len(c)>0:
                if ord(c)==0xaa:
                    c=self.sport.read()
                    if ord(c)==0x55:
                        CT=ord(self.sport.read())
                        LSN=ord(self.sport.read())
                        FSA=int.from_bytes(self.sport.read(2),byteorder='little')
                        LSA=int.from_bytes(self.sport.read(2),byteorder='little')
                        CS=int.from_bytes(self.sport.read(2),byteorder='little')
                        data_stream=[]
                        for i in range(LSN):
                            Si=int.from_bytes(self.sport.read(2),byteorder='little')
                            dist=Si>>2
                            Acorrect= 0 if (dist==0) else atan(19.16*(dist-90.15)/(dist*90.15))
                            F=(FSA>>1)/64.0
                            L=(LSA>>1)/64.0
                            Angle=F+((L-F)/(LSN))*(i)-Acorrect
                            data_stream=data_stream+[(Angle,dist)]
                        if CT==1:
                            self.DataPack=self.__newpack__.copy()                                                  
                            self.__newpack__=data_stream.copy()
                        elif CT==0:
                            self.__newpack__=self.__newpack__+data_stream
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_KP_PLUS or event.key==pygame.K_EQUALS:
                        if self.scale>3:
                            self.scale/=2
                    if event.key == pygame.K_KP_MINUS or event.key==pygame.K_MINUS:
                        if self.scale<40:
                            self.scale*=2
                    if event.key == pygame.K_KP8 or event.key == pygame.K_8 :
                        self.scale=40
                    if event.key == pygame.K_KP1 or event.key==pygame.K_1:
                        self.scale=5
                    if event.key == pygame.K_KP2 or event.key==pygame.K_2:
                        self.scale=10
                    if event.key == pygame.K_KP3 or event.key==pygame.K_3:
                        self.scale=15
                    if event.key == pygame.K_KP4 or event.key==pygame.K_4:
                        self.scale=20                            
                    if event.key == pygame.K_KP5 or event.key==pygame.K_5:
                        self.scale=25
                    if event.key == pygame.K_KP6 or event.key==pygame.K_6:
                        self.scale=30
                    if event.key == pygame.K_KP7 or event.key==pygame.K_7:
                        self.scale=35
                    if event.key == pygame.K_g:
                        self.gridstyle=(self.gridstyle+1)%2
                    if event.key == pygame.K_d:
                        self.backcolor=(0,0,0)
                        self.gridcolor=(80,80,255)
                        self.lessergridcolor=(0,0,96)
                        self.gridcentercolor=(255,255,255)
                    if event.key == pygame.K_b:
                        self.backcolor=(255,255,255)
                        self.gridcolor=(0,0,128)
                        self.lessergridcolor=(192,192,255)
                        self.gridcentercolor=(0,0,0)
                    if event.key == pygame.K_ESCAPE:
                        self.running=False
                        flag_lidar_running=False
            self.screen.fill(self.backcolor)
            self.__put_grid__(self.gridstyle)
            if len(self.DataPack)==0:
                text=self.Hugefont.render(f"NO DATA RECEIVED",False,(255,0,0))
                origin=coord_diff((320,320),size_scale(text.get_size(),2))
                self.screen.blit(text,origin)        
            else:                     
                for polar in self.DataPack:
                    coord=polar_to_point(polar,scale=self.scale)
                    pygame.draw.circle(self.screen,(255,0,0),coord,3)

            pygame.display.update()
    
if __name__ == '__main__':
    ports_available=list(map(lambda n:n.name,list(serial.tools.list_ports.comports())))
    print ('Selecione a porta a ser utilizada:')
    for n in ports_available:
        print(f"{n[-1]}: {n}")
    print("0: Sair do programa")
    n=input()
    if ('COM'+n in ports_available):    
        L=Lidar('COM'+n)
        L.run()
        
    