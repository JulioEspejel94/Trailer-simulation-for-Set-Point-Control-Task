#Simulation for articulated robot
from math import * #to avoid prefix math.
import numpy as np #to use matrix
import matplotlib.pyplot as plt
from functions1 import inner_loop
from functions1 import kinematics
from functions1 import draw_robot_with_trailer
from functions1 import draw_robot
from functions1 import draw_trailer
from functions1 import set_point_controller

#global variables
N_iterations = 350
main_counter = 0 #to initialize iterations

#Vehicle physical parameters
N_trailers=1

L_1 = 0.8

L_h1 = 0

L_i=[0,L_1]
L_hi=[0,L_h1]

dt=0.1
theta_aux=[0,0] # initial condition, variable used in inner_loop function 
delta_k = 1 

#Main class definition
class Zero:
    def __init__(self): #It is done only the first iteration
        #declaration and initial values of important variables
        self.pose=np.zeros([N_trailers+1,1]) #x_0,y_0,theta_0 of the robot extracted from the robot sensors
        self.q=np.zeros([N_trailers+3,1]) #x_2,y_2,theta_2,theta_1,theta_0 of the N-trailer
        self.q_est=np.zeros([N_trailers+3,1]) #estimation of N-trailer after fusion
        self.q_est_0=np.zeros([N_trailers+2,1]) #estimation of tractor pose
        self.input=np.zeros([2,1]) #[w,v].T
        self.vehicle_param=np.array([L_i,L_hi])
        

robot = Zero()  
robot.q=np.array([[0],[0],[pi/2],[pi/2]])   #q_i [x1 y1 01 02]
robot.q_est=robot.q
robot.q_est_0=robot.pose
q_real=robot.q

#Buffers for the graphics and animation
robot_positions = []
trailer_positions = []
theta_robot = []
t_final = []

error_0 = [] 
error_x = []
error_y = []


theta_0b = []
theta_1b = []
tsh = []

v_lineal = []
w_angular = []


# Inicial configuration for the plot
fig, ax = plt.subplots()


# Main Script
while main_counter <= N_iterations:
    print(main_counter)
    #Example position goal
    #goal = np.array([[4],[4],[pi/4]])
    #goal = np.array([[-4],[4],[3*pi/4]])
    #goal = np.array([[-4],[-4],[pi/4]])
    goal = np.array([[4],[-4],[3*pi/4]])
    motion = 0
    if motion == 0:
        phi, error = set_point_controller(motion, robot.q_est, goal)

        [u_0,theta_aux]=inner_loop(robot.q_est,phi,robot.vehicle_param,1,theta_aux,motion,delta_k)

        #tractor states
        theta_1=q_real[2][0]
        theta_0=q_real[3][0]
        x0=q_real[0][0]+L_1*cos(theta_1)+L_h1*cos(theta_0)
        y0=q_real[1][0]+L_1*sin(theta_1)+L_h1*sin(theta_0)
        robot.q_est_0=np.array([[x0],[y0],[theta_0]])

        q_kN=kinematics(robot.q_est,u_0,dt,robot.vehicle_param,N_trailers,delta_k)
        robot.q_est=q_kN

        inputw=u_0[0][0]
        inputv=u_0[1][0]
        inputs=np.array([[inputw],[inputv]])

    elif motion == 1:
        phi, error = set_point_controller(motion, robot.q_est, goal)

        [u_0,theta_aux]=inner_loop(robot.q_est,phi,robot.vehicle_param,1,theta_aux,motion,delta_k)

        #tractor states
        theta_1=q_real[2][0]
        theta_0=q_real[3][0]
        x0=q_real[0][0]+L_1*cos(theta_1)+L_h1*cos(theta_0)
        y0=q_real[1][0]+L_1*sin(theta_1)+L_h1*sin(theta_0)
        robot.q_est_0=np.array([[x0],[y0],[theta_0]])


        q_kN=kinematics(robot.q_est,u_0,dt,robot.vehicle_param,N_trailers,delta_k)
        robot.q_est=q_kN


        inputw=u_0[0][0]
        inputv=u_0[1][0]
        inputs=np.array([[inputw],[inputv]])

    #print(sqrt((q_real[0][0]-robot.q_est_0[0][0])**2 + (q_real[1][0]-robot.q_est_0[1][0])**2))


    q_real=kinematics(q_real,inputs,dt,robot.vehicle_param,N_trailers,1)

    e_x=goal[0][0]-q_real[0][0]
    e_y=goal[1][0]-q_real[1][0]
    e_th=goal[2][0]-q_real[2][0]
    e_th = (e_th+ np.pi) % (2 * np.pi) - np.pi
    error=np.array([[e_x],[e_y],[e_th]])

    #Cycles to seconds
    tsh = main_counter*dt


    robot_positions.append((robot.q_est_0[0][0], robot.q_est_0[1][0]))
    trailer_positions.append((q_real[0][0], q_real[1][0]))
    v_lineal.append((inputs[1][0], tsh))
    w_angular.append((inputs[0][0], tsh))
    error_0.append((error[2][0], tsh)) 
    error_x.append((error[0][0], tsh))
    error_y.append((error[1][0], tsh))
    theta_0b.append((q_real[2][0], tsh))
    theta_1b.append((robot.q_est_0[2][0], tsh))
    t_final.append((goal[2][0], tsh))

    #Arrays
    robot_positions1 = np.array(robot_positions)
    trailer_positions1 = np.array(trailer_positions)
    theta_0a = np.array(theta_0b)
    theta_1a = np.array(theta_1b)
    t_final1 = np.array(t_final)
    error_0_1 = np.array(error_0)
    error_x_1 = np.array(error_x)
    error_y_1 = np.array(error_y)
    tsh_1 = np.array(tsh)
    v_lineal1 = np.array(v_lineal)
    w_angular1 = np.array(w_angular)


    # Clear for the next iteration
    ax.clear()
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)


    # Quiver goal
    length = 1
    dx = length * np.cos(goal[2][0])
    dy = length * np.sin(goal[2][0])
    ax.quiver(goal[0][0], goal[1][0], dx, dy, angles='xy', scale_units='xy', scale=3)

    plt.plot(robot_positions1[:, 0], robot_positions1[:, 1], 'b--', label='Robot')
    plt.plot(trailer_positions1[:, 0], trailer_positions1[:, 1], 'g--', label='Trailer')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('1 Trailer Simulation')
    plt.grid(True)
    draw_robot_with_trailer(ax, robot.q_est_0, q_real)
    plt.pause(0.01)


    main_counter=main_counter+1

plt.show()


plt.figure(figsize=(18, 6))
#Error
plt.subplot(1, 3, 1)
plt.plot(error_x_1[:,1], error_x_1[:,0], 'r--', label='error_x [m]')
plt.plot(error_y_1[:,1], error_y_1[:,0], 'g--', label='error_y [m]')
plt.plot(error_0_1[:,1], error_0_1[:,0], 'b--', label='error_theta [rad]')
plt.xlabel('Time (s)')
plt.title('Error')
plt.legend()
plt.grid(True)
plt.ylim(-5, 5)

#Theta evolution
plt.subplot(1, 3, 2)
plt.plot(theta_1a[:,1], theta_1a[:,0], 'g--', label='theta_0 [rad]')
plt.plot(theta_0a[:,1], theta_0a[:,0], 'r--', label='theta_1 [rad]')
plt.xlabel('Time (s)')
plt.title('Theta Angles')
plt.legend()
plt.grid(True)
plt.ylim(0, 4)

#Velocities
plt.subplot(1, 3, 3)
plt.plot(v_lineal1[:,1], v_lineal1[:,0], 'r--', label='v_lineal [m/s]')
plt.plot(w_angular1[:,1], w_angular1[:,0], 'b--', label='w_angular [rad/s]')
plt.ylabel('V W')
plt.xlabel('t(s)')
plt.title('Velocities')
plt.legend()
plt.grid(True)
plt.ylim(-1.5, 1.5)


plt.tight_layout()
plt.show()
