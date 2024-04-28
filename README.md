# Trailer-simulation-for-Set-Point-Control-Task
Pyton simulation of the backward and forward control of a unicycle robot with a trailer in an on-axle configuration.


The present project is to achieve stabilisation at a desired point of an N-trailer system in on-axle configuration, as can be seen in the attached image.


![Diapositiva3](https://github.com/JulioEspejel94/Trailer-simulation-for-Set-Point-Control-Task/assets/159551087/aa648a92-dc06-4fd2-8d30-56788d1a7a58)


To achieve this task, a controller based on the VFO (Vector Field Orientation) method was implemented in the outer-loop, scaling the control system to the trailer, since the main objective is to converge the position of the trailer at the desired point, either with the reverse or forward strategy.
As first point we have the Script file called "function1", in this script we have the necessary functions to execute the simulation, these functions are the following ones:

## Functions Script

**Set Point Controller**


In this function the VFO control law is applied, having 3 arguments for its application, `motion`, `pose`, `goal`, each one indicates the name of the variable needed.
* `motion` must have a value of 1 or 0, represents Forward Strategy and 0 represents Backward strategy,
* `pose` is the current position of the trailer, theoretically represented by $q_i$ = [ $x_0$ $y_0$ $\theta_0$ $\theta_1$ ].
* `goal` is the desired point, represented by $q_d$ = [ $x_d$ $y_d$ $\theta_d$ $\theta_d$ ].


**Inner loop**


Aims to transform the velocities given by the controller to be applicable to the mobile robot, in the same way as the controller, this function outputs the linear and angular velocities to be applied directly to the kinematics.
The input arguments for this function are `q`, `V`, `vehicle_param`, `N_trailers`, `y_a`, `motion`:


* `Q` represents the current position of the robot, which is defined from a start and updated at each cycle of the simulation.


* `V` are the velocities given by the outer-loop controller, which need to be scaled to the trailer. 


* `Vehicle_param` are the physical parameters of the system, mainly the length of hitches.


* `N_trailers` represents the number of trailers, for this case it is specifically 1.


* `y_a` is an array to store the values of the beta angles of the model.


**Kinematics**

The kinematics represents how the values given by the controller will act within the N-trailer model or system, the return of this function is the new position after one iteration cycle. 

**Draw robot with trailer, draw robot, draw trailer**

Together, these three functions are only intended to plot the trailer and the robot in a simple way, they have no return and the inputs will always be the current positions of the robot and the trailer.


## Main script

Once the functions used to calculate each iteration of the simulation have been defined, the main code is created, in which each iteration is carried out, the system parameters are defined and the behaviour of the model and how it reacts to the controller is observed graphically.

First we define the variables and parameters of the simulation, among the most important variables to be defined are: Total number of iterations  `N_iterations`, counter of the main loop initialised to 0 `main_counter`, physical parameters of the vehicle such as the number of trailers `N_trailers` and length of the couplings `L_1` `L_h1`, array to store the angles of the robot and the trailer `theta_aux`.

The Robot class is also defined, which includes arrays to store the values of the simulation, position, controller inputs, physical parameters, etc. Additionally, there are the arrays in which the simulation data will be stored for later graphing.
Define the initial position of the trailer, this will have as data the positions of the trailer and the theta angles of the trailer and the robot, both in radians. 

Once the variables, parameters and buffers to be used have been defined, the main loop is found, which will have a maximum number of iterations of `N_iterations`, at the beginning of this the objective point and the strategy to be used (Forward or Backward motion) are defined, then the control scheme is applied, where the functions previously defined in the fucntions script are used, the control scheme is the next one:


![Diapositiva1](https://github.com/JulioEspejel94/Trailer-simulation-for-Set-Point-Control-Task/assets/159551087/df46dca0-57da-4953-93b8-b3d77ecced63)


The scheme is simple, first the outer-loop has as input the desired position and the current position, the output of this outer-loop called $u_0$ is the set of linear and angular velocities, which is necessary to scale to the trailer, this is done through the inner-loop, which in this case will return the values of $u_1$, this new set of velocities is applied to the kinematics, which models the response of the entire system to the applied velocities.

At the end of the loop the current positions of the robot and trailer are plotted and updated, and the values of these are stored in the arrays created specifically for this task, since it is necessary to have all the values to make the graphs and the analysis of the results.



## Examples


Below are the results of different cases in which the simulation is applied, varying the target position as well as the strategy implemented to reach that point,

* Example 1
![Diapositiva4](https://github.com/JulioEspejel94/Trailer-simulation-for-Set-Point-Control-Task/assets/159551087/b6629bc6-2861-462e-9e42-e53da97e17b3)

* Example 2
![Diapositiva5](https://github.com/JulioEspejel94/Trailer-simulation-for-Set-Point-Control-Task/assets/159551087/b5c2beef-2077-40e9-8b6a-8dfd59b1ee27)


* Example 3
![Diapositiva6](https://github.com/JulioEspejel94/Trailer-simulation-for-Set-Point-Control-Task/assets/159551087/95abb924-cb6d-4313-907b-07cb84eea26b)


* Example 4
![Diapositiva7](https://github.com/JulioEspejel94/Trailer-simulation-for-Set-Point-Control-Task/assets/159551087/5a1807a9-d3e0-4fc4-882c-658fd9ed395e)

  

It is important to note that the strategy used will be defined as follows: motion = 1 represents forward motion, while motion = 0 represents backward motion.
It is important to note that this simulation does not take into account disturbances or errors within the system, so different results are likely to be expected in a real case or situation. 	



