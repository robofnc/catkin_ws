#!/usr/bin/env python
# coding:utf-8
# license removed for brevity

import rospy#import ros module
from std_msgs.msg import String#import ros messeage type
import matplotlib.pyplot as plt
import math
import numpy as np
from numpy import sin,cos
from geometry_msgs.msg import Point#import ro messeage type
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from ros_start.msg import f3darray

##rosの宣言##################################
rospy.init_node('talker')
pub = rospy.Publisher('a_point',f3darray,queue_size=100)
rate = rospy.Rate(100)   
#############################################

show_animation = True


class Node:
    
    def __init__(self, x, y,t, cost, pind):
        self.x = x
        self.y = y
        self.t = t
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x)+ ","  + str(self.y) + ","+ str(self.t) + ","+ str(self.cost) + "," + str(self.pind)

def final_definition(rx,ry,rt):

    rex,rey=[],[]

    ngs=len(rx)
    for i in range(ngs):
        sx=rx[i]
        sy=ry[i]
        xwid=3
        ywid=88


        ori=np.array([[0],[0]])#orijin position
        p1=np.array([[sx],[sy]])
        #define p2,p3,p4 reference p1
        p2=np.array([[sx+xwid],[sy]])
        p3=np.array([[sx+xwid],[sy+ywid]])
        p4=np.array([[sx],[sy+ywid]])
        ##print("p1,p2,p3,p4",p1,p2,p3,p4)
        Pori=np.array([[sx+(xwid/2.0)],[sy+(ywid/2.0)]])

        #define  Unit Direction vector ut1,ut2,ut3,ut4
        t0=p1-ori
        Tori=Pori-p1
        t1=p2-p1
        t2=p3-p2
        t3=p4-p3
        t4=p1-p4
        #print("t1",t1)
        #vol0=np.linalg.norm(t0)
        vol1=np.linalg.norm(t1)#np.linalg.nor→ベクトルの大きさを計算
        vol2=np.linalg.norm(t2)
        vol3=np.linalg.norm(t3)
        vol4=np.linalg.norm(t4)
        volori=np.linalg.norm(Tori)
        #print("nt1",vol1)
        #ut0=t0/vol0
        nt1=t1/vol1
        nt2=t2/vol2
        nt3=t3/vol3
        nt4=t4/vol4
        ntori=Tori/volori
        #print("ut1,t2,ut3,ut4",ut1,ut2,ut3,ut4)

        seg12,seg23,seg34,seg41=[],[],[],[]#create list
        
        deg=rt[i]
        rad=(deg*np.pi/180)
        rot=np.array([[np.cos(rad),-np.sin(rad)],[np.sin(rad),np.cos(rad)]])
        np1=np.array([[sx],[sy]])
        np2=np1+np.dot(rot,nt1)*xwid
        np3=np2+np.dot(rot,nt2)*ywid
        np4=np3+np.dot(rot,nt3)*xwid
        np5=np4+np.dot(rot,nt4)*ywid
        npori=np1+np.dot(rot,ntori)*math.sqrt((Pori[0]-p1[0])**2+(Pori[1]-p1[1])**2)



        rex.append(npori[0])
        rey.append(npori[1])

        """
        for i in range(int(xwid+1)):
            seg12.append(np1+np.dot(rot,nt1)*i)
            plt.plot(seg12[i][0],seg12[i][1],"xc")
        #print("seg12",seg12)

        for i in range(int(ywid+1)):
            seg23.append(np2+np.dot(rot,nt2)*i)
            plt.plot(seg23[i][0],seg23[i][1],"xc")
        print("seg23",seg23)

        for i in range(int(xwid+1)):
            seg34.append(np3+np.dot(rot,nt3)*i)
            plt.plot(seg34[i][0],seg34[i][1],"xc")
        #print("seg34",seg34)

        for i in range(int(ywid+1)):
            seg41.append(np4+np.dot(rot,nt4)*i)
            plt.plot(seg41[i][0],seg41[i][1],"xc")
        #print("seg41",seg41)
        """

        """cheak p1=p5    
            if np.all(p1)==np.all(p5):
                
                    print(True)
        """
        """
        plt.plot(np1[0],np1[1],"yo")
        plt.plot(np2[0],np2[1],"ro")
        plt.plot(np3[0],np3[1],"bo")
        plt.plot(np4[0],np4[1],"go") 
        """
        
        plt.plot([np1[0],np2[0]],[np1[1],np2[1]],"lime")
        plt.plot([np2[0],np3[0]],[np2[1],np3[1]],"lime")
        plt.plot([np3[0],np4[0]],[np3[1],np4[1]],"lime")
        plt.plot([np4[0],np1[0]],[np4[1],np1[1]],"lime")  
        plt.plot(npori[0],npori[1],"xr") 
        #print("deg,p1,p2,p3,p4",deg,np1,np2,np3,np4)
        """
        for i in so25:
            for j in seg23:
                if i in j:
                    print("collision",seg23)
        """


    return (rex,rey,0,0)

def initial_definition(dx,dy,t):


    sx=dx
    sy=dy
    xwid=3
    ywid=88


    ori=np.array([[0],[0]])#orijin position
    p1=np.array([[sx],[sy]])
    #define p2,p3,p4 reference p1
    p2=np.array([[sx+xwid],[sy]])
    p3=np.array([[sx+xwid],[sy+ywid]])
    p4=np.array([[sx],[sy+ywid]])
    ##print("p1,p2,p3,p4",p1,p2,p3,p4)

    #define  Unit Direction vector ut1,ut2,ut3,ut4
    t0=p1-ori
    t1=p2-p1
    t2=p3-p2
    t3=p4-p3
    t4=p1-p4
    #print("t1",t1)
    #vol0=np.linalg.norm(t0)
    vol1=np.linalg.norm(t1)#np.linalg.nor→ベクトルの大きさを計算
    vol2=np.linalg.norm(t2)
    vol3=np.linalg.norm(t3)
    vol4=np.linalg.norm(t4)
    #print("nt1",vol1)
    #ut0=t0/vol0
    nt1=t1/vol1
    nt2=t2/vol2
    nt3=t3/vol3
    nt4=t4/vol4
    #print("ut1,t2,ut3,ut4",ut1,ut2,ut3,ut4)

    seg12,seg23,seg34,seg41=[],[],[],[]#create list
    
    deg=t
    rad=(deg*np.pi/180)
    rot=np.array([[np.cos(rad),-np.sin(rad)],[np.sin(rad),np.cos(rad)]])
    np1=np.array([[sx],[sy]])
    np2=np1+np.dot(rot,nt1)*xwid
    np3=np2+np.dot(rot,nt2)*ywid
    np4=np3+np.dot(rot,nt3)*xwid
    np5=np4+np.dot(rot,nt4)*ywid


    """
    for i in range(int(xwid+1)):
        seg12.append(np1+np.dot(rot,nt1)*i)
        plt.plot(seg12[i][0],seg12[i][1],"xc")
    #print("seg12",seg12)

    for i in range(int(ywid+1)):
        seg23.append(np2+np.dot(rot,nt2)*i)
        plt.plot(seg23[i][0],seg23[i][1],"xc")
    print("seg23",seg23)

    for i in range(int(xwid+1)):
        seg34.append(np3+np.dot(rot,nt3)*i)
        plt.plot(seg34[i][0],seg34[i][1],"xc")
    #print("seg34",seg34)

    for i in range(int(ywid+1)):
        seg41.append(np4+np.dot(rot,nt4)*i)
        plt.plot(seg41[i][0],seg41[i][1],"xc")
    #print("seg41",seg41)
    """

    """cheak p1=p5    
        if np.all(p1)==np.all(p5):
            
                print(True)
    """
    """
    plt.plot(np1[0],np1[1],"yo")
    plt.plot(np2[0],np2[1],"ro")
    plt.plot(np3[0],np3[1],"bo")
    plt.plot(np4[0],np4[1],"go") 
    """
    """
    plt.plot([np1[0],np2[0]],[np1[1],np2[1]],"c")
    plt.plot([np2[0],np3[0]],[np2[1],np3[1]],"c")
    plt.plot([np3[0],np4[0]],[np3[1],np4[1]],"c")
    plt.plot([np4[0],np1[0]],[np4[1],np1[1]],"c")  
    """
    #print("deg,p1,p2,p3,p4",deg,np1,np2,np3,np4)
    """
    for i in so25:
        for j in seg23:
            if i in j:
                print("collision",seg23)
    """


    return (np1,np2,np3,np4)


def collision_cheak(o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11,o12,o13,o14,np1,np2,np3,np4):

    p1=np1
    p2=np2
    p3=o1
    p4=o2
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    
    np121=tc1*tc2<=0 and td1*td2<=0
        
    p1=np1
    p2=np2
    p3=o2
    p4=o3
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np122=tc1*tc2<=0 and td1*td2<=0

    p1=np1
    p2=np2
    p3=o3
    p4=o4
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np123=tc1*tc2<=0 and td1*td2<=0
    
    p1=np1
    p2=np2
    p3=o4
    p4=o14
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np124=tc1*tc2<=0 and td1*td2<=0

    p1=np1
    p2=np2
    p3=o6
    p4=o5
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np125=tc1*tc2<=0 and td1*td2<=0

    p1=np1
    p2=np2
    p3=o8
    p4=o7
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np126=tc1*tc2<=0 and td1*td2<=0   

    p1=np1
    p2=np2
    p3=o9
    p4=o10
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np127=tc1*tc2<=0 and td1*td2<=0   


    p1=np1
    p2=np2
    p3=o10
    p4=o13
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np128=tc1*tc2<=0 and td1*td2<=0   

    p1=np1
    p2=np2
    p3=o11
    p4=o12
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np129=tc1*tc2<=0 and td1*td2<=0   

    p1=np1
    p2=np2
    p3=o8
    p4=o9
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np1210=tc1*tc2<=0 and td1*td2<=0   

    np120=np121 or np122 or np123 or np124 or np125 or np126 or np127 or np128 or np129 or np1210


    p1=np2
    p2=np3
    p3=o1
    p4=o2
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    
    np231=tc1*tc2<=0 and td1*td2<=0
        
    p1=np2
    p2=np3
    p3=o2
    p4=o3
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np232=tc1*tc2<=0 and td1*td2<=0

    p1=np2
    p2=np3
    p3=o3
    p4=o4
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np233=tc1*tc2<=0 and td1*td2<=0
    
    p1=np2
    p2=np3
    p3=o4
    p4=o14
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np234=tc1*tc2<=0 and td1*td2<=0

    p1=np2
    p2=np3
    p3=o5
    p4=o6
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np235=tc1*tc2<=0 and td1*td2<=0

    p1=np2
    p2=np3
    p3=o8
    p4=o9
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np236=tc1*tc2<=0 and td1*td2<=0   

    p1=np2
    p2=np3
    p3=o9
    p4=o10
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np237=tc1*tc2<=0 and td1*td2<=0

    p1=np2
    p2=np3
    p3=o10
    p4=o13
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np238=tc1*tc2<=0 and td1*td2<=0

    p1=np2
    p2=np3
    p3=o11
    p4=o12
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np239=tc1*tc2<=0 and td1*td2<=0

    p1=np2
    p2=np3
    p3=o8
    p4=o7
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np2310=tc1*tc2<=0 and td1*td2<=0                

    np230=np231 or np232 or np233 or np234 or np235 or np236 or np237 or np238 or np239 or np2310


    p1=np3
    p2=np4
    p3=o1
    p4=o2
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    
    np341=tc1*tc2<=0 and td1*td2<=0
        
    p1=np3
    p2=np4
    p3=o2
    p4=o3
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np342=tc1*tc2<=0 and td1*td2<=0

    p1=np3
    p2=np4
    p3=o3
    p4=o4
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np343=tc1*tc2<=0 and td1*td2<=0
    
    p1=np3
    p2=np4
    p3=o4
    p4=o14
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np344=tc1*tc2<=0 and td1*td2<=0

    p1=np3
    p2=np4
    p3=o5
    p4=o6
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np345=tc1*tc2<=0 and td1*td2<=0

    p1=np3
    p2=np4
    p3=o8
    p4=o7
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np346=tc1*tc2<=0 and td1*td2<=0 
    
    p1=np3
    p2=np4
    p3=o8
    p4=o9
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np347=tc1*tc2<=0 and td1*td2<=0

    p1=np3
    p2=np4
    p3=o9
    p4=o10
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np348=tc1*tc2<=0 and td1*td2<=0

    p1=np3
    p2=np4
    p3=o10
    p4=o13
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np349=tc1*tc2<=0 and td1*td2<=0

    p1=np3
    p2=np4
    p3=o11
    p4=o12
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np3410=tc1*tc2<=0 and td1*td2<=0


    np340=np341 or np342 or np343 or np344 or np345 or np346 or np347  or np348 or np349 or np3410



    p1=np4
    p2=np1
    p3=o1
    p4=o2
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np411=tc1*tc2<=0 and td1*td2<=0
        
    p1=np4
    p2=np1
    p3=o2
    p4=o3
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np412=tc1*tc2<=0 and td1*td2<=0

    p1=np4
    p2=np1
    p3=o3
    p4=o4
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np413=tc1*tc2<=0 and td1*td2<=0
    
    p1=np4
    p2=np1
    p3=o4
    p4=o14
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np414=tc1*tc2<=0 and td1*td2<=0

    p1=np4
    p2=np1
    p3=o5
    p4=o6
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np415=tc1*tc2<=0 and td1*td2<=0

    p1=np4
    p2=np1
    p3=o7
    p4=o8
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np416=tc1*tc2<=0 and td1*td2<=0   

    p1=np4
    p2=np1
    p3=o8
    p4=o9
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np417=tc1*tc2<=0 and td1*td2<=0

    p1=np4
    p2=np1
    p3=o9
    p4=o10
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np418=tc1*tc2<=0 and td1*td2<=0

    p1=np4
    p2=np1
    p3=o10
    p4=o13
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np419=tc1*tc2<=0 and td1*td2<=0

    p1=np4
    p2=np1
    p3=o11
    p4=o12
    tc1 = (p1[0] - p2[0]) * (p3[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p3[0])
    tc2 = (p1[0] - p2[0]) * (p4[1] - p1[1]) + (p1[1] - p2[1]) * (p1[0] - p4[0])
    td1 = (p3[0] - p4[0]) * (p1[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p1[0])
    td2 = (p3[0] - p4[0]) * (p2[1] - p3[1]) + (p3[1] - p4[1]) * (p3[0] - p2[0])
    np4110=tc1*tc2<=0 and td1*td2<=0

    np410=np411 or np412 or np413 or np414 or np415 or np416 or np417 or np418 or np419 or np4110





    #print("np120,np230,np340,np410",np120,np230,np340,np410)
    odf=np120 or np230 or np340 or np410
    return odf
    
def obstacle_map(o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11,o12,o13,o14):

    """
    #draw line
    plt.plot([o1[0],o3[0]],[o1[1],o3[1]],"r")
    plt.plot([o2[0],o5[0]],[o2[1],o5[1]],"r")
    plt.plot([o6[0],o4[0]],[o6[1],o4[1]],"r")
    plt.plot([o7[0],o8[0]],[o7[1],o8[1]],"r")
    plt.plot([o9[0],o11[0]],[o9[1],o11[1]],"r")
    plt.plot([o10[0],o12[0]],[o10[1],o12[1]],"r")
    """

    plt.plot([o1[0],o2[0]],[o1[1],o2[1]],"r")
    plt.plot([o3[0],o4[0]],[o3[1],o4[1]],"r")
    plt.plot([o2[0],o3[0]],[o2[1],o3[1]],"r")
    plt.plot([o4[0],o14[0]],[o4[1],o14[1]],"r")
    plt.plot([o6[0],o5[0]],[o6[1],o5[1]],"r")


    plt.plot([o7[0],o8[0]],[o7[1],o8[1]],"r")
    plt.plot([o9[0],o8[0]],[o9[1],o8[1]],"r")
    plt.plot([o10[0],o9[0]],[o10[1],o9[1]],"r")
    plt.plot([o10[0],o13[0]],[o10[1],o13[1]],"r")
    plt.plot([o11[0],o12[0]],[o11[1],o12[1]],"r")
    
    """
    #define  Unit Direction vector
    to13=o1-o3
    to25=o5-o2
    to64=o6-o4
    to87=o8-o7
    to119=o11-o9
    to1012=o12-o10

    vol01=np.linalg.norm(to13)#np.linalg.nor→ベクトルの大きさを計算
    vol02=np.linalg.norm(to25)
    vol03=np.linalg.norm(to64)  
    vol04=np.linalg.norm(to87)
    vol05=np.linalg.norm(to119)
    vol06=np.linalg.norm(to1012)
    #print("vol01",vol01)
    
    o13=to13/vol01
    o25=to25/vol02
    o64=to64/vol03
    o78=to87/vol04
    o911=to119/vol05
    o1012=to1012/vol06

    #print("o13,o25,o64,o78,o911,o1012",o13,o25,o64,o78,o911,o1012)

    #create obstacle line segment
    so13,so25,so64,so78,so911,so1012=[],[],[],[],[],[]
    for i in range(int(vol01)):
        so13.append(o3+o13*i)
    #print("so13",so13)
    for i in range(int(vol02)):
        so25.append(o2+o25*i)
    print("so25",so25)
    for i in range(int(vol03)):
        so64.append(o4+o64*i)
    #print("so64",so64)
    for i in range(int(vol04)):
        so78.append(o7+o78*i)
    #print("so78",so78)
    for i in range(int(vol05)):
        so911.append(o9+o911*i)
    #print("so911",so911)
    for i in range(int(vol06)):
        so1012.append(o10+o1012*i)
    #print("so1012",so1012)
    """
    #return (so13,so25,so64,so78,so911,so1012)
    return(0,0,0,0,0,0,0)

def calc_fianl_path(ngoal, closedset, reso):
    # generate final course
    rx, ry,rt = [ngoal.x * reso], [ngoal.y * reso],[ngoal.t*reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        rt.append(n.t * reso)
        pind = n.pind
      ## a=np.array(rx)
    """
    df=len(rx)
    print("nodeLength=",df)
    print("xpoint=",rx,"ypoint=",ry)
    """
    return rx,ry,rt

def a_star_planning(sx, sy,st, gx, gy,gt, ox, oy, reso, rr):

    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """
    """
    #def obstacle point
    o1=np.array([[30],[40+85]])
    o2=np.array([[30],[20+85+6.5]])
    o3=np.array([[30],[20+85]])
    o4=np.array([[36],[20+85]])
    o5=np.array([[36],[20+85+6.5]])
    o6=np.array([[36],[40+85]])
    o7=np.array([[30],[20]])
    o8=np.array([[30],[0]])
    o9=np.array([[30],[20-6.5]])
    o10=np.array([[36],[20]])
    o11=np.array([[36],[20-6.5]])
    o12=np.array([[36],[0]])
    """
    #def obstacle point
    o1=np.array([[23-10],[40+85]])
    o2=np.array([[23-10],[20+85]])
    o3=np.array([[30],[20+85]])
    o4=np.array([[30],[20+85+6.5]])
    o5=np.array([[36],[20+85]])
    o6=np.array([[36],[40+85]])

    o7=np.array([[23],[0]])
    o8=np.array([[22.9],[20]])
    o9=np.array([[30],[20]])
    o10=np.array([[30],[20-6.5]])
    o11=np.array([[36],[20]])
    o12=np.array([[36],[0]])

    o13=np.array([[36],[20-6.5]])
    o14=np.array([[36],[20+85+6.5]])


    nstart = Node(round(sx / reso), round(sy / reso), 0.0,0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, 0.0,-1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]
    ##print("ox",ox)
    ##print("oy",oy)

    minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)
    #so13,so25,so64,so78,so911,so1012=obstacle_map(o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11,o12)#go initial_definition func
    so13,so25,so64,so78,so911,so1012,so1013=obstacle_map(o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11,o12,o13,o14)#go initial_definition func
    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]
        ##print("currentx=",current.x,"currenty=",current.y)

        # show graph
        if show_animation:
            plt.plot(current.x * reso, current.y * reso, "xc")
        
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)
        
        #if (abs(current.x - ngoal.x))<0.4 and (abs(current.y == ngoal.y))<0.4 :
        if current.x == ngoal.x and current.y == ngoal.y :
            #print("Find goal,",current.x,ngoal.x,current.y,ngoal.y,current.t,ngoal.t)

            ngoal.pind = current.pind
            ngoal.cost = current.cost
            x=current.x
            y=current.y
            t=current.t
            """
            np1,np2,np3,np4=initial_definition(x,y,t)
            plt.plot([np1[0],np2[0]],[np1[1],np2[1]],"c")
            plt.plot([np2[0],np3[0]],[np2[1],np3[1]],"c")
            plt.plot([np3[0],np4[0]],[np3[1],np4[1]],"c")
            plt.plot([np4[0],np1[0]],[np4[1],np1[1]],"c")  
            """
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(motion)):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.t + motion[i][2],
                        current.cost + motion[i][3], c_id)
            n_id = calc_index(node, xw, minx, miny)

            #print("current.x,current.y,current.t",i,current.x,current.y,current.t)
            #print("node.x,node.y,node.t",node.x,node.y,node.t)
            
            dx=node.x
            dy=node.y
            t=node.t
        
            np1,np2,np3,np4=initial_definition(dx,dy,t)
            #print("deg,p1,p2,p3,p4",np1,np2,np3,np4)

            #odf=collision_cheak(o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11,o12,np1,np2,np3,np4)
            odf=collision_cheak(o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11,o12,o13,o14,np1,np2,np3,np4)

            if n_id in closedset:
                continue

            if not verify_node(node,minx, miny, maxx, maxy):
                continue
            
            if  (odf==True):
                #print(np1,np2,np3,np4)
                continue

            if n_id not in openset:
                openset[n_id] = node  # Discover a new node
            else:
                if openset[n_id].cost >= node.cost:
                    # This path is the best until now. record it!
                    openset[n_id] = node

    rx, ry ,rt= calc_fianl_path(ngoal, closedset, reso)

    return rx, ry,rt


def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d


def verify_node(node,minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    print("minx:", minx)
    print("miny:", miny)
    print("maxx:", maxx)
    print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    print("xwidth:", xwidth)
    print("ywidth:", ywidth)
    """
    # obstacle map generation
    obmap = [[False for i in range(int(xwidth))] for i in range(int(ywidth))]
    for ix in range(int(xwidth)):
        x = ix + minx
        for iy in range(int(ywidth)):
            y = iy + miny
            ##print("ix",x,"iy", y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] =True
                    break
    ##print("obmap=",obmap)
    """
    return  minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, dθ ,cost
    tm=1.0
    motion = [[1*tm, 0, 0, 1],[1*tm, 0, 1, 1],[1*tm, 0, -1, 1],
              [0, 1*tm, 0, 1],[0, 1*tm, 1, 1],[0, 1*tm, -1, 1],
              [-1*tm, 0, 0, 1],[-1*tm, 0, 1*tm, 1],[-1*tm, 0, -1, 1],
              [0, -1*tm, 0, 1],[0, -1*tm, 1,1],[0, -1*tm, -1, 1],
              [-1*tm, -1*tm, 0, math.sqrt(2)],[-1*tm, -1*tm, 1, math.sqrt(2)],[-1*tm, -1*tm, -1, math.sqrt(2)],
              [-1*tm, 1*tm, 0, math.sqrt(2)],[-1*tm, 1*tm, 1, math.sqrt(2)],[-1*tm, 1*tm, -1, math.sqrt(2)],
              [1*tm, -1*tm, 0, math.sqrt(2)],[1*tm, -1*tm, 1, math.sqrt(2)],[1*tm, -1*tm, -1, math.sqrt(2)],
              [1*tm, 1*tm, 0, math.sqrt(2)],[1*tm, 1*tm, 1, math.sqrt(2)],[1*tm, 1*tm, -1, math.sqrt(2)],]

    return motion

    
def inverse(rx,ry,rt): ##rx,ry list inverse
    invrx=rx[::-1]
    invry=ry[::-1]
    invrt=rt[::-1]
            #drxx=rx[-(i+2)]-rx[-(i+1)]
           #dryy=ry[-(i+2)]-ry[-(i+1)]
            ##dry=int((invrx[(i+1)]-invrx[(i)]))
            #atans = math.degrees(math.atan(dryy/drxx))
           ## print("rx=",invrx,"ry=",invry)
    return(invrx,invry,invrt)


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 12
    # [m]
    sy = 21.0 # [m]
    st = 0
    gx = 32
    gy = 18
    gt = 0 # [m]
    grid_size =  1.0 # [m]
    robot_size = 1.0 # [m]

    ox, oy = [], []

    for i in range(50):
        ox.append(i)
        oy.append(0.0)
    for i in range(125):
        ox.append(50.0)
        oy.append(i)
    for i in range(50):
        ox.append(i)
        oy.append(125.0)
    for i in range(126):
        ox.append(0.0)
        oy.append(i)
    """
    for i in range(15):
        ox.append(40.0)
        oy.append(60.0 - i)
    for i in range(15):
        ox.append(30)
        oy.append(60.0-i)
    for i in range(10):
        ox.append(30)
        oy.append(i)
    for i in range(10):
        ox.append(40)
        oy.append(i)
    for i in range(10):
        ox.append(40-i)
        oy.append(8)
    for i in range(10):
        ox.append(40-i)
        oy.append(51)    
   
    osr=ox,oy
    print("osr",osr)
    """


    
    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry,rt = a_star_planning(sx, sy,st, gx, gy,gt, ox, oy, grid_size, robot_size)

    invrx,invry,invrt=inverse(rx,ry,rt)

    ded=len(invrx)
    print ("ded",ded)
    """
    print ("inverserx=",invrx,"inverse=ry",invry)
    poly = Polygon((0,0), (0,3), (4,4), (3,0), (2,2))
    point = Point(1,2)
    poly.encloses_point(point)
    print("sdf",poly.encloses_point(point))
    """
    #print(rx,ry,rt)

    print("rx",invrx)
    print("ry",invry)
    print("rt",invrt)


    

    nsp1,nsp2,nsp3,nsp4=final_definition(invrx,invry,invrt)

    nsp1.append(100)
    nsp2.append(100)
    invrt.append(100)

    print("centrx",nsp1)
    print("centry",nsp2)
    print("centrt",invrt)
    
    #return invrx,invry,invrt


    

    #hello_str = Point()
    hello_str=f3darray()
    while not rospy.is_shutdown():
        """
        nss=len(invrx)
        for i in range(nss):
        """

        
        hello_str.x =nsp1#[325.0,2682,6832,2296,8865,7796,6955,8236]
        hello_str.y =nsp2
        hello_str.t =invrt
        """
        hello_str.y=invry[i]
        hello_str.z=invrt[i]
        """
        pub.publish(hello_str)
    plt.show()
    rate.sleep()
    
    
"""
def publish():
    
    publisher = rospy.Publisher('/foo', FooSensor, queue_size=1)
    value = FooSensor(value=[100, 110, 120])
    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        publisher.publish(value)
        r.sleep()

publish()
"""

if __name__ == '__main__':
    main()


