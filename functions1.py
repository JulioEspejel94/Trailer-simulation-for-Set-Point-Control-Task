import numpy as np
from math import *

#The following fuctions are used in the main script for the N-trailer simulation

#Set point control task
def set_point_controller(motion, pose, goal):

    distThresh = 0.001

    n = 0.4
    ka = 2
    kp = 1

    if motion == 1:
        sigma = 1
    elif motion == 0:
        sigma = -1

    e_y = goal[1][0] - pose[1][0]
    e_x = goal[0][0] - pose[0][0]

    distance = sqrt(e_x**2 + e_y**2)

    hx = kp*e_x + (-n * sigma*cos(goal[2][0])*distance)
    hy = kp*e_y + (-n * sigma*sin(goal[2][0])*distance)

    d_hx = -kp*(pose[0][0]) + (n*sigma*(pose[0][0]*e_x + pose[1][0]*e_y)/(distance))*cos(goal[2][0])    
    d_hy = -kp*(pose[1][0]) + (n*sigma*(pose[0][0]*e_x + pose[1][0]*e_y)/(distance))*sin(goal[2][0])

    theta_a = atan2(sigma*hy , sigma*hx)

    if sigma == 1:
        theta_a = (theta_a)%(2*np.pi)

    e_theta = theta_a - pose[2][0]

    d_theta = (d_hy*hx - hy*d_hx)/((hx)**2+(hy)**2)
    d_theta = (d_theta)%(2*pi)


    if distance > distThresh:
        if sigma == 1:
            w = (ka*(e_theta))
            v = (hx*cos(pose[2][0]) + hy*sin(pose[2][0]))
        elif sigma == -1:
            w = (ka*(e_theta))
            v = (hx*cos(pose[2][0]) + hy*sin(pose[2][0]))
    else:
        w = 0
        v = 0

    vel = np.array([[w], [v]])
    error = goal - pose[:3]

    return vel, error


#Inner loop function
def inner_loop(q,V,vehicle_param,N_trailers,y_a,motion,delta):

    if motion == 1:
        sigma = 1
    elif motion == 0:
        sigma = -1

    E = 0.001

    theta_1=q[2][0]
    theta_0=q[3][0]
    Betas=[0,theta_0-theta_1]
    vehicle_param_local=np.zeros([2,N_trailers+1])
    vehicle_param_local[1,:]=vehicle_param[1,:]*delta
    vehicle_param_local[0,:]=vehicle_param[0,:]*delta

    j=N_trailers #J = 1

    while j>0:
        L_h=vehicle_param_local[1][j]
        L=vehicle_param_local[0][j]
        beta=Betas[j]

        if L_h==0: 
            if j==N_trailers:
                k=10 #1 10 
            else:
                k=5 #5 50 

            #Atan2c calculation
            phi=atan2(sigma*L*V[0][0],sigma*V[1][0])
            phi_past=atan2(sin(y_a[j]),cos(y_a[j]))

            delta_phi=phi-phi_past

            if delta_phi > pi: 
                delta_theta=delta_phi-2*pi

            elif delta_phi<-pi:
                delta_theta=delta_phi+2*pi
            else:
                delta_theta=delta_phi

            y_a[j] = y_a[j]+delta_theta #list

            #V = np.array([[(-L*cos(beta)*V[0][0])/(E)+(sin(beta)*V[1][0])/(E)],[L*sin(beta)*V[0][0]+cos(beta)*V[1][0]]])

            w1 = k*(y_a[j]-beta)+V[0][0]

            v1 = sigma*np.absolute(L*sin(beta)*V[0][0]+cos(beta)*V[1][0])

            V=np.array([[w1/8],[v1/5]])



        else:
            V=np.array([[-(L/L_h)*cos(beta)*V[0][0]+(1/L_h)*sin(beta)*V[1][0]],[L*sin(beta)*V[0][0]+cos(beta)*V[1][0]]])      
        j=j-1
    return V,y_a


#Kinematics function
def kinematics(q,u,dt,vehicle_param,N_trailers,delta):
    #Trailer state
    theta_1=q[2][0]
    theta_0=q[3][0]

    vehicle_param_local=np.zeros([2,N_trailers+1])
    vehicle_param_local[1,:]=vehicle_param[1,:]*delta
    vehicle_param_local[0,:]=vehicle_param[0,:]*delta

    L_h1=vehicle_param_local[1][1]
    L_1=vehicle_param_local[0][1]

    S1 = [ -L_h1*cos(theta_1)*sin(theta_0 - theta_1), cos(theta_1)*cos(theta_0-theta_1)]
    S2 = [    L_h1*sin(theta_1)*sin(theta_0-theta_1), sin(theta_1)*cos(theta_0-theta_1)]
    S3 = [          (-L_h1*cos(theta_0-theta_1))/L_1,          sin(theta_0-theta_1)/L_1]
    S4 = [                                         1,                                 0]

    S = np.array([S1,S2,S3,S4])  #(4,1)
    dq = np.dot(S,u)
    q= q + dq * dt
    return q


#Draw fucntions
def draw_robot(ax, q, label):
    x0 = q[0][0]
    y0 = q[1][0] 
    theta_0 = q[2][0]
    base_triangle = np.array([[0.4, 0], [-0.2, -0.2], [-0.2, 0.2], [0.4, 0]])
    R = np.array([[np.cos(theta_0), -np.sin(theta_0)],
                  [np.sin(theta_0), np.cos(theta_0)]])

    rotated_triangle = (R @ base_triangle.T).T + ([x0, y0])
    ax.plot(rotated_triangle[:, 0], rotated_triangle[:, 1], 'b-')
    ax.plot(x0, y0, 'ko')

def draw_trailer(ax, q, label):
    x1 = q[0][0]
    y1 = q[1][0] 
    theta_1 = q[2][0]

    base_rectangle = np.array([[-0.4, 0.2], [0.4, 0.2], [0.4, -0.2], [-0.4, -0.2], [-0.4, 0.2]])
    R = np.array([[np.cos(theta_1), -np.sin(theta_1)],
                  [np.sin(theta_1), np.cos(theta_1)]])

    rotated_rectangle = (R @ base_rectangle.T).T + ([x1, y1])
    ax.plot(rotated_rectangle[:, 0], rotated_rectangle[:, 1], 'g-')
    ax.plot(x1, y1, 'ko')

def draw_robot_with_trailer(ax, q_robot, q_trailer):
    draw_robot(ax, q_robot, '')
    draw_trailer(ax, q_trailer, '')
    ax.plot([q_robot[0][0], q_trailer[0][0]], [q_robot[1][0], q_trailer[1][0]], 'k--')






