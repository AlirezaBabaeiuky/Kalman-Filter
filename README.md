# Kalman_Filter
Extended Kalman Filter for Nonlinear Systems 
EKF is designed in MATLB without using any built-in functions to estimate the system state. 
It is important to note that KF (Kalman Filter) is a type of State Observer commonly used in modern control techniques. 
It is decent to briefly recall the concepts in Controllability and Observability: 
Controllability: a system is called controllable if it is possible to drive any state of the system to any desired state in a given finite time. 
Oberveability: a system is called observable if it is possible to derive or infer internal states of a system from external measurements. 
If the system is controllable, you can theoretically find a State FeedBack Gain matrix that drives the Closed-Loop Location of the poles (eigen values of the State Matrix) to the desired location. 
In most cases, a system is not fully observable (sensors like: encoders and accelerometers are pricy and expensive), or it is not feasible to locate the sensors at some locatoins. 
In such cases, State Observers are of great importance to infer internal dynamics from external measurements. 
Kalman Filter and Leunberger are among the most-commonly-used observers. Kalman filter is a type of Discrete State Observer. It is designed based on the recursive and iterative process to estimate the 
proper gain of the filter to estimate the states. 
Essentially, Kalman filter design is similar to State FeedBack Control techniques, which lead to Duality theorem. 
![image](https://github.com/user-attachments/assets/a80c3243-8ac7-4876-9f8e-d2d5661f23fb)
In the above figure, it is obvious that 'Ke' is the estimator (observer) gain which is defined via the same procedure for the state feedback gain. 
When using the State Observer, the control signal is -ke (state observer gain) multiplied to the ESTIMATED STATE (u_ctrl = - Ke * x_hat). If there is no observer then the control signal is -Ke * x. 
Other important note is: 
x_hat_dot = A*x_hat + B * u + Ke * (y-y_hat); where: y=C*x and y_hat = C*x_hat  
Scripting the Kalman Filter in MATLAB to estimate the position state of a nonlinear mass-spring-damper is enclosed to this repositoroy. 
![image](https://github.com/user-attachments/assets/d2b300b6-8ed8-4870-b13f-2b622fcb29ee)
The designed EKF has following RMSE as a decent metrics to assess its capability: 
RMSE for position: 1.225e-9
RMSE for velocity: 0.1143

