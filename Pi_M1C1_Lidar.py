# -*- coding: utf-8 -*-
"""
Created on Sun Jul 11 13:26:17 2021

@author: hlpp
"""

import os
import serial
from serial.tools.list_ports import comports
from math import atan
import pygame
from math import pi, sin, cos, floor

coord_diff=lambda x,y:(x[0]-y[0],x[1]-y[1])
size_scale=lambda x,s:(x[0]/s,x[1]/s)
             
def polar_to_point(polar,center=(320,320), scale=25.0, calibration=1):
    ang=polar[0]*pi/180.0
    dist=polar[1]/scale*calibration
#    return (center[0]+dist*sin(ang),center[1]-dist*cos(ang)) #original position
    return (center[0]-dist*sin(ang),center[1]+dist*cos(ang)) #"natural" position (180 rotation)
            
class Lidar ():
    def __init__(self, port):
        try:
            self.sport=serial.Serial(port,115200,timeout=1)
            self.__initOK__=True
            print ("press [ESC] to exit")
        except:
            print (f"Could not open {port}")
            self.__initOK__=False
        self.ws=960  #multiple of 80, please!!!!
        self.hws=self.ws/2
        self.max_scale=8000/self.hws #endo of scale is 8000mm; scale is in mm/pixel
        self.scale=self.max_scale
        self.dot_size=3
        self.gridstyle=0
        
        #default to darkmode        
        self.backcolor=(0,0,0)
        self.gridcolor=(80,80,255)
        self.lessergridcolor=(0,0,96)
        self.gridcentercolor=(255,255,255)
        
        # #default to lightmode
        # self.backcolor=(255,255,255)
        # self.gridcolor=(0,0,128)
        # self.lessergridcolor=(192,192,255)
        # self.gridcentercolor=(0,0,0)

        self.__newpack__=[]
        self.DataPack=self.__newpack__.copy()
        self.__init_screen__()
        self.__put_grid__()
        pygame.display.update()
        
    def __init_screen__(self):
        pygame.init()
        self.screen=pygame.display.set_mode((self.ws,self.ws))
        pygame.font.init()
        pygame.display.set_caption('Lidar data')
        self.screen.fill(self.backcolor)
        self.myfont=pygame.font.SysFont('Calibri',16)
        self.Hugefont=pygame.font.SysFont('Calibri',60,bold=True)
        self.Grid_Surface=pygame.Surface((self.ws,self.ws))
        self.__create_grid__(0)
     

    def __create_grid__(self, style=0):
        self.Grid_Surface.fill(self.backcolor)
        major_step=floor(self.hws/8)
        minor_step=floor(major_step/5)
        if style==0: #polar (default)
            for i in range(8):
                pygame.draw.circle(self.Grid_Surface,self.gridcolor,(self.hws,self.hws),(i+1)*major_step,width=1)
                for j in range(minor_step,major_step,minor_step):
                    pygame.draw.circle(self.Grid_Surface,self.lessergridcolor,(self.hws,self.hws),(i+1)*major_step-j,width=1)
            for i in range(0,360,90):
                for j in range (15,90,15):
                    pygame.draw.line(self.Grid_Surface,self.lessergridcolor,(self.hws,self.hws),(self.hws+self.hws*sin(pi*(i+j)/180.0),self.hws+self.hws*cos(pi*(i+j)/180.0)),1)           
        else: #rectangular
            for i in range(0,self.ws+1,minor_step):
                color=self.lessergridcolor
                pygame.draw.line(self.Grid_Surface, color, (0,i),(self.ws, i))                
                pygame.draw.line(self.Grid_Surface, color, (i,0),(i,self.ws))                
            for i in range(0,self.ws+1,major_step):
                color=self.gridcolor
                pygame.draw.line(self.Grid_Surface, color, (0,i),(self.ws, i))                
                pygame.draw.line(self.Grid_Surface, color, (i,0),(i,self.ws))                
        i=self.hws
        color=self.gridcentercolor
        pygame.draw.line(self.Grid_Surface, color, (0,i),(self.ws, i))                
        pygame.draw.line(self.Grid_Surface, color, (i,0),(i,self.ws))                

        for i in range(8):
            dist=(i+1)*1000*(self.scale/self.max_scale)
            text=self.myfont.render(f"{dist} mm",False,self.gridcolor)
            self.Grid_Surface.blit(text,(self.hws+10,self.hws-((i+1)*major_step)))
            self.Grid_Surface.blit(text,(self.hws+10,self.hws+((i+1)*major_step)))        
     
    def __put_grid__(self):
        self.screen.blit(self.Grid_Surface,(0,0))
        
                             
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
                            self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_KP_MINUS or event.key==pygame.K_MINUS:
                        if self.scale<self.max_scale:
                            self.scale*=2
                            self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_KP8 or event.key == pygame.K_8 :
                        self.scale=self.max_scale
                        self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_KP1 or event.key==pygame.K_1:
                        self.scale=self.max_scale/8 #*1/8
                        self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_KP2 or event.key==pygame.K_2:
                        self.scale=self.max_scale/4 #*2/8
                        self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_KP3 or event.key==pygame.K_3:
                        self.scale=self.max_scale*3/8
                        self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_KP4 or event.key==pygame.K_4:
                        self.scale=self.max_scale/2 #*4/8                     
                        self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_KP5 or event.key==pygame.K_5:
                        self.scale=self.max_scale*5/8
                        self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_KP6 or event.key==pygame.K_6:
                        self.scale=self.max_scale*3/4 #*6/8
                        self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_KP7 or event.key==pygame.K_7:
                        self.scale=self.max_scale*7/8
                        self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_g:
                        self.gridstyle=(self.gridstyle+1)%2
                        self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_d:
                        self.backcolor=(0,0,0)
                        self.gridcolor=(80,80,255)
                        self.lessergridcolor=(0,0,96)
                        self.gridcentercolor=(255,255,255)        
                        self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_b:
                        self.backcolor=(255,255,255)
                        self.gridcolor=(0,0,128)
                        self.lessergridcolor=(192,192,255)
                        self.gridcentercolor=(0,0,0)
                        self.__create_grid__(self.gridstyle)
                    if event.key == pygame.K_RIGHTBRACKET:
                        if self.dot_size<6:
                            self.dot_size+=1
                    if event.key == pygame.K_LEFTBRACKET:
                        if self.dot_size>1:
                            self.dot_size-=1                            
                    if event.key == pygame.K_ESCAPE:
                        self.running=False
                        flag_lidar_running=False
            self.screen.fill(self.backcolor)
            self.__put_grid__()
            if len(self.DataPack)==0:
                text=self.Hugefont.render(f"NO DATA RECEIVED",False,(255,0,0))
                origin=coord_diff((self.hws,self.hws),size_scale(text.get_size(),2))
                self.screen.blit(text,origin)        
            else:                     
                for polar in self.DataPack:
                    coord=polar_to_point(polar,center=(self.hws,self.hws),scale=self.scale)
                    pygame.draw.circle(self.screen,(255,0,0),coord,self.dot_size)

            pygame.display.update()
    
if __name__ == '__main__':

    ports_available=list(map(lambda n:n.device,list(serial.tools.list_ports.comports())))
    ports_available.sort()      
        
    print ('Select port to be used:')
    for i,n in enumerate(ports_available):
        print(f"{i+1}: {n}")
    print("Press any other key to exit this program")
    try:
        n=int(input())
        if os.name=='nt':
            print("In Windows, you need to set focus to the graphic window manually")    
        if n in range(1, len(ports_available)+1):
            L=Lidar(ports_available[n-1])
            L.run()
    except:
        pass
    print ("Bye!")    