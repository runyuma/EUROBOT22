# -*- coding: utf-8 -*-
"""
Created on Sat Mar 26 14:37:13 2022

@author: Lin YANG
"""

import numpy as np
import math


def cosd(theta):
    return math.cos(theta*math.pi/180)
def sind(theta):
    return math.sin(theta*math.pi/180)
def cos(theta):
    return math.cos(theta)
def sin(theta):
    return math.sin(theta)
def atan2(y,x):
    return math.atan2(y,x)

def T01(theta1):
    fuck= np.array([[cosd(theta1),0,sind(theta1),0],[sind(theta1),0,-cosd(theta1),0],[0,1,0,0],[0,0,0,1]])
    return fuck
def T12(theta2):
    a2=0.104
    fuck= np.array([[cosd(theta2),-sind(theta2),0,a2*cosd(theta2)],[sind(theta2),cosd(theta2),0,a2*sind(theta2)],[0,0,1,0],[0,0,0,1]])
    return fuck    
def T23(theta2):
    a3=0.089
    fuck= np.array([[cosd(theta2),-sind(theta2),0,a3*cosd(theta2)],[sind(theta2),cosd(theta2),0,a3*sind(theta2)],[0,0,1,0],[0,0,0,1]])
    return fuck       
def T34(theta2):
    fuck= np.array([[cosd(theta2),-sind(theta2),0,0],[sind(theta2),cosd(theta2),0,0],[0,0,1,0],[0,0,0,1]])
    return fuck 
def T45():
    fuck= np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
    return fuck 
def T56(theta2):
    fuck= np.array([[cosd(theta2),-sind(theta2),0,0],[sind(theta2),cosd(theta2),0,0],[0,0,1,0],[0,0,0,1]])
    return fuck 
def T6e(theta2):
    ae=0
    de=0.2
    fuck= np.array([[1,0,0,ae],[0,1,0,0],[0,0,1,de],[0,0,0,1]])
    return fuck 

def T0e(theta1,theta2,theta3,theta4,theta5,theta6):
    T02=np.dot(T01(theta1),T12(theta2))
    T03=np.dot(T02,T23(theta3))
    T04=np.dot(T03,T34(theta4))
    T05=np.dot(T04,T45())
    T06=np.dot(T05,T56(theta5))
    T0end=np.dot(T06,T6e(theta6))
    return T0end

def five_dof_ikine(fk_T):
    a2 = 0.104; a3 = 0.089; ae = 0; de = 0.2; 
    nx = fk_T[1-1, 1-1]
    ox = fk_T[1-1, 2-1]
    ax = fk_T[1-1, 3-1]
    px = fk_T[1-1, 4-1]
    ny = fk_T[2-1, 1-1]
    oy = fk_T[2-1, 2-1]
    ay = fk_T[2-1, 3-1]
    py = fk_T[2-1, 4-1]
    nz = fk_T[3-1, 1-1]
    oz = fk_T[3-1, 2-1]
    az = fk_T[3-1, 3-1]
    pz = fk_T[3-1, 4-1]
    theta1 = atan2(py - ny*ae - ay*de, px - nx*ae - ax*de)
    theta5 = atan2(sin(theta1)*nx - cos(theta1)*ny, sin(theta1)*ox - cos(theta1)*oy)
    m = px - nx*ae - ax*de
    n = py - ny*ae - ay*de
    t = pz - nz*ae - az*de
    c3 = ((cos(theta1)**2)*(m**2) + (sin(theta1)**2)*(n**2) + 2*sin(theta1)*cos(theta1)*m*n + (t**2) - (a2**2) - (a3**2)) / (2*a2*a3)
    if c3>1:
        c3=1
    theta3_1 = atan2(((1-(c3**2))**0.5), c3)
    theta3_2 = atan2(-(1-(c3**2))**0.5, c3)

    c2_1 = ((a3*cos(theta3_1) + a2)*(cos(theta1)*m + sin(theta1)*n) + a3*sin(theta3_1)*t) / (((a3*cos(theta3_1) + a2)**2) + (a3**2)*((sin(theta3_1))**2))
    s2_1 = ((a3*cos(theta3_1) + a2)*t - a3*sin(theta3_1)*(cos(theta1)*m + sin(theta1)*n)) / (((a3*cos(theta3_1) + a2)**2) + (a3**2)*((sin(theta3_1))**2))
    # print('c3',c2_1)
    c2_2 = ((a3*cos(theta3_2) + a2)*(cos(theta1)*m + sin(theta1)*n) + a3*sin(theta3_2)*t) / (((a3*cos(theta3_2) + a2)**2) + (a3**2)*((sin(theta3_2))**2))
    s2_2 = ((a3*cos(theta3_2) + a2)*t - a3*sin(theta3_2)*(cos(theta1)*m + sin(theta1)*n)) / (((a3*cos(theta3_2) + a2)**2) + (a3**2)*((sin(theta3_2))**2))
    theta2_1 = atan2(s2_1, c2_1)
    theta2_2 = atan2(s2_2, c2_2)

    theta4_1 = atan2(az, cos(theta1)*ax + sin(theta1)*ay) - theta3_1 - theta2_1
    theta4_2 = atan2(az, cos(theta1)*ax + sin(theta1)*ay) - theta3_2 - theta2_2
    ik_T = np.array([[theta1,theta2_1,theta3_1,theta4_1, theta5],[theta1, theta2_2, theta3_2, theta4_2, theta5]])   
    # print('ik_T',ik_T)
    h1=ik_T[0,:]*180/math.pi
    h2=ik_T[1,:]*180/math.pi
    solution1=np.array([h1[0]-180,180-h1[1],-h1[2],-h1[3],180+h1[4]])
    solution2=np.array([h2[0]-180,180-h2[1],-h2[2],-h2[3],180+h2[4]])
    solution3=np.array([h1[0],h1[1],h1[2],h1[3],h1[4]])
    solution4=np.array([h2[0],h2[1],h2[2],h2[3],h2[4]])
    # print('solution1',solution1)
    # print('solution2',solution2)
    # print('solution3',solution3)
    # print('solution4',solution4)
    
    adjust_solu1=np.array([solution1[0],90-solution1[1],solution1[2],solution1[3],solution1[4]])
    adjust_solu2=np.array([solution2[0],90-solution2[1],solution2[2],solution2[3],solution2[4]])
    adjust_solu3=np.array([solution3[0],90-solution3[1],solution3[2],solution3[3],solution3[4]])
    adjust_solu4=np.array([solution4[0],90-solution4[1],solution4[2],solution4[3],solution4[4]])
    sum1=sum(abs(adjust_solu1))
    sum2=sum(abs(adjust_solu2))
    sum3=sum(abs(adjust_solu3))
    sum4=sum(abs(adjust_solu4))
    result_slu=[solution1,solution2,solution3,solution4]
    result=[sum1,sum2,sum3,sum4]
    temp_min=sum1
    temp_i=0
    for i in range(0,4):
        if result[i] <= temp_min:
            temp_min=result[i]
            temp_i=i
    return result_slu[temp_i]
    

# print(five_dof_ikine(T_matrix))



def T_goal(x,y,z,A,B,Y):
    eularA= np.array([[cosd(A),-sind(A),0],[sind(A),cosd(A),0],[0,0,1]])
    eularB= np.array([[cosd(B),0,sind(B)],[0,1,0],[-sind(B),0,cosd(B)]])
    eularY= np.array([[1,0,0],[0,cosd(Y),-sind(Y)],[0,sind(Y),cosd(Y)]])
    RAB=np.dot(eularA,eularB)
    RABY=np.dot(RAB,eularY)
    e=np.append(RABY,np.array([[x],[y],[z]]),axis=1)
    T=np.append(e,np.array([[0,0,0,1]]),axis=0)
    return T



def quat2angle(w,x,y,z):
    r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    p = math.asin(2*(w*y-z*x))
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
    angleR = r*180/math.pi
    angleP = p*180/math.pi
    angleY = y*180/math.pi
    print (angleR)
    print (angleP)
    print (angleY)
    return [angleR,angleP,angleY]

def Camera2base(Z,Y,x,y,z):
    eularZ= np.array([[cosd(Z),-sind(Z),0],[sind(Z),cosd(Z),0],[0,0,1]])
    eularY= np.array([[cosd(Y),0,sind(Y)],[0,1,0],[-sind(Y),0,cosd(Y)]])
    Rtemp=np.dot(eularY,eularZ)
    e=np.append(Rtemp,np.array([[x],[y],[z]]),axis=1)
    T=np.append(e,np.array([[0,0,0,1]]),axis=0)    
    return T



def inverse_angle(R):
    b=atan2(-R[2,0] , (R[0,0]**2+R[1,0]**2)**0.5);
    print(b*180/math.pi)
    if b*180/math.pi == 90:
        a=0
        y=atan2(R[0,1],R[1,1])
    elif b*180/math.pi == -90:
        a=0
        y=-atan2(R[0,1],R[1,1])            
    else:
        a = atan2( (R[1,0]/cos(b)) , (R[0,0]/cos(b)))
        y = atan2( (R[2,1]/cos(b)) , (R[2,2]/cos(b)))

    a=a*180/math.pi
    b=b*180/math.pi
    y=y*180/math.pi
    X=R[0,3]
    Y=R[1,3]
    Z=R[2,3]
    return [X,Y,Z,a,b,y]




# print('test_target is 4*4 \n',test_target)
# result_input6=five_dof_ikine(test_target)


# TorF=np.isnan(result_input6).any()
# print('init_result_input6\n',result_input6)
# loop=1
# while TorF == True:
#     test_target=T_goal(x,y,z,A,B-loop,Y)
#     loop+=1
#     test_target
#     result_loop=five_dof_ikine(test_target)
#     TorF=np.isnan(result_loop).any()
#     if result_loop[1] < 0:
#         TorF=True
#     elif result_loop[1] > 180:
#         TorF=True
            
# print(loop)    
# result_loop=result_input6
        
# print('result_loop',result_loop)

# adjust_result=[0,result_loop[4],-result_loop[3],-result_loop[2],90-result_loop[1],-result_loop[0]]
# print('adjust_result\n',adjust_result)



def new_loop_solution(T):

    # print('init_result_input6\n',result_loop)
    loop=1
    #[X,Y,Z,a,b,y]=inverse_angle(T)
    X=T[0,3]
    Y=T[1,3]
    Z=T[2,3]
    a = 0
    b = -15
    y = 180
    #result_loop=five_dof_ikine(T)
    #TorF=np.isnan(result_loop).any()
    TorF=1
    print('T',T)
    print('[X,Y,Z,a,b,y]',[X,Y,Z,a,b,y])
    while TorF == True:
        test_target=T_goal(X,Y,Z,a,b-loop,y)
        loop+=1
        result_loop=five_dof_ikine(test_target)
        TorF=np.isnan(result_loop).any()
        if result_loop[1] < 0:
            TorF=True
        elif result_loop[1] > 180:
            TorF=True
    # print('loop',loop)
    # print('result_loop',result_loop)
    adjust_result=[0,result_loop[4],-result_loop[3],-result_loop[2],90-result_loop[1],-result_loop[0]+10]
    print(adjust_result)
    return adjust_result



if __name__ == "__main__":
    'y is allowed to add/minus'
    # ok=0
    # while ok==1:
        
    T_matrix=T0e(10,30,-45,-45,20,0)
    print('T_matrix \n',T_matrix)  
    # result_5=five_dof_ikine(T_matrix)

    result_5=five_dof_ikine(T_matrix)
    assume=Camera2base(-90,-135,0.1,0,0)
    # print('assume_T\n',assume)
    # print('assume_T\n',assume)


    print('result5 \n',result_5)
    [X,Y,Z,a,b,y]=inverse_angle(T_matrix)
    print([X,Y,Z,a,b,y])


    x=0.27
    y=0.04
    z=-0.144
    A=0
    B=0
    Y=180
    test_target=T_goal(x,y,z,A,B,Y)
    new_result=new_loop_solution(T_matrix)
    print(new_result)
