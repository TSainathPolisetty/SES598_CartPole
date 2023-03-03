# Inverted Pendulum LQR control

## Launch gazebo model using 
`roslaunch invpend_control load_invpend.launch`

## PID
### PID controller parameter tuning 
Operate rqt_reconfigure to modify PID values
launch rqt_reconfigure with the following command

`rosrun rqt_reconfigure rqt_reconfigure`

### Demo: Inbuilt PID controller
Demo includes setting pole to horizontal and publishing velocity commands.
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/5TH1VlKVSfs/0.jpg)](https://youtu.be/VQCreAFs9Ag)

## LQR control
`controllqr.py` in `/invpend_control/scripts/` is to be run to execute LQR control.
Check `/inverted_pendulum_EOM_Lagrangian.pdf` for derivation of Equations of motion using Lagrangian method and derivation of A, B, in the equation:
`Xd = Ax + Bu` 

Goal is set to

`[[1],[0],[0],[0]]`

### Demo of LQR control on inverted pendulum. Goal : `[[1],[0],[0],[0]]`
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/5TH1VlKVSfs/0.jpg)](https://youtu.be/KuO8ucu_CFo)




## Plot of the cart position, velocity, pole position and velocity
![plot2](https://user-images.githubusercontent.com/122844128/222645277-47719d90-dcbc-4a3c-a21c-cf3f632e1e6e.png)

LQR controller converges on the set goal quite well. 

Cost matrix choosen is 
`Q = np.diag([1, 1, 10, 100]);`

The state cost matrix (Q) is chosen based on the sensitivity analysis of each state variable to keep the system in the linearized window. The input cost matrix (R) is chosen to prioritize low actuator effort for specific actuator states. However, for the inverted pendulum system, controllability is more critical than conserving actuator effort. Hence, a lower value is chosen for the R matrix to achieve better controllability.
`R = np.diag([0.1])`

