# -*- coding: utf-8 -*-
"""
Created on Sun Apr  4 14:30:43 2021

@author: venkatesh surampally
"""
from pywebio.output import *
from pywebio.input import * 
import time
import numpy as np
import matplotlib.pyplot as plt
import math
from mpl_toolkits import mplot3d

def findtrajectory(data1,data2,n,data3,vZ_wind):
    B = 1                                                          
    L = data1['L']
    D = data1['D']                     
    SA = data2['SA']                                      
    M = data2['M']

    k1 = 0.1386727598                     
    k2 = 0.00596838162                    
    k3 = 0.00645287298                    
    k = (k1*B*D**2 + k2*L*D + k3*n*SA)/M

    g = 9.81
    pi=math.pi
    
    x = []
    y = []
    z=[]
    V = []
    theta=[]
    
    v=data3['v']
    theta1=(data3['angle'])*pi/180
    rho=1.225
    delta_t = 0.0005
    sum_x = 0
    sum_y = 0
    v_z=0
    vz=[]
    z=[]
    z_pos=0
    CD_Z=1.5
    time1=[]
    t=0
    for i in range(5000):
        Vx = v*math.cos(theta1)
        Vy = v*math.sin(theta1)
        delta_x = (Vx)*delta_t - 0.5*k*(Vx**2)*(delta_t)**2
        delta_y = (Vy)*delta_t - 0.5*(k*(Vy**2) + g)*(delta_t)**2
        x.append(sum_x)
        y.append(sum_y)
        sum_x = sum_x + delta_x
        sum_y = sum_y + delta_y
        V.append(v)
        t+=delta_t
        time1.append(t)
        v=((delta_x/delta_t)**2  + (delta_y/delta_t)**2)**0.5
        theta1=math.atan(delta_y/delta_x)
        theta.append(theta1)
        f_z=0.5*rho*((vZ_wind-v_z)**2)*CD_Z*SA
        v_z+=f_z*delta_t/M
        z_pos+=0.5*f_z*delta_t**2/M+v_z*delta_t
        vz.append(v_z)
        z.append(z_pos)
        if(sum_y<=0):
            break
    plt.figure(figsize=(20,5))

    plt.subplot(1, 2, 1)
    plt.plot(x,y)
    plt.title('Trajectory in xy plane')
    plt.xlabel('x')
    plt.ylabel('y')

    plt.subplot(1, 2, 2)
    plt.plot(x,z)
    plt.title('Trajectory in xz plane')
    plt.xlabel('x')
    plt.ylabel('z')
    plt.savefig('trajectory2d.png')
    
    # importing mplot3d toolkits, numpy and matplotlib
    plt.figure(figsize=(20,5))
    plt.plot(time1,vz)
    plt.title('Velocity along the z vs time')
    plt.xlabel('time')
    plt.ylabel('V_z')
    plt.savefig('vzvstime.png')
    
    plt.figure(figsize=(20,5))
    plt.plot(time1,V)
    plt.title('Velocity in the xy plane vs time')
    plt.xlabel('time')
    plt.ylabel('V')
    plt.savefig('vvstime.png')
    

    fig = plt.figure()
    plt.figure(figsize=(10,10))
    ax = plt.axes(projection ='3d')
    ax.plot3D(x, z, y, 'green')
    ax.set_title('Trajectory')
    plt.xlabel('x')
    plt.ylabel('z')
    ax.set_zlabel('y')
    plt.savefig('trajectory3d.png')
    plt.show()
    
    put_processbar('bar')
    for i in range(1, 11):
        set_processbar('bar', i / 10)
        time.sleep(0.1)
        
    put_table([
    ['Parameter', 'Value'],
    ['Max Height', max(y)],
    ['Time of Flight', max(time1)],    
    ['Range', max(x)],
    ['Drift', max(z)]
    ])
        
if __name__ == '__main__':
    put_markdown('## Hello there')

    put_text("Use this app to find the trajectory of an arrow.")
    
    
    with popup("Note"):
        put_text("This is a software which relys upon the data  from the research conducted by group 3!")
    
    condition = select("Choose the type of sorroundings", ['Calm', 'Air along the direction of z'])
    
    with popup("Caution"):
        put_text("Please enter the data in SI units only!")
        
    data1=input_group('Enter the data',[input('Enter the length of the arrow.', name='L',type=FLOAT),input('Enter the diameter of the shaft.',name='D', type=FLOAT)])
    data2=input_group('Enter the data',[input('Enter the mass of the arrow.', name='M',type=FLOAT),input('Enter the surface area of 1 fletching.',name='SA', type=FLOAT)])
    
    n = select("Choose the number of fletchings", [0,1,2,3,4,5,6,])
    
    data3=input_group('Enter the launch settings',[input('Enter the launch velocity.', name='v',type=FLOAT),input('Enter the launch angle in degrees.',name='angle', type=FLOAT)])
    
    vz_wind=0
    if(condition=='Air along the direction of z'):
        vz_wind=input('Enter the velocity of air along the z direction. ',type = FLOAT)
        
    put_text(f"We are solving the trajectory. Please wait until it is calculated!")
    
    out = findtrajectory(data1,data2,n,data3,vz_wind)
    
    def btn_click(btn_val):
        put_markdown("> You have clicked `%s` button" % btn_val)
        if(btn_val=='Trajectory'):
            put_image(open('trajectory2d.png', 'rb').read(),width='1500px')
            put_image(open('trajectory3d.png', 'rb').read(),width='3000px')
        else:
            put_image(open('vzvstime.png', 'rb').read(),width='2000px')
            put_image(open('vvstime.png', 'rb').read(),width='2000px')
    
    put_text(f"Choose the plots which you wanted to see.")
    put_buttons(['Trajectory','Velocity'], onclick=btn_click)
    
    put_markdown("Finished!")