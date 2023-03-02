## Inverted Pendulum LQR control

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


## Plot of cart position, velocity and Pole angular position and angular velocity
![alt txt](invpend_control/scripts/plot1.png "Plot 1")
LQR controller converges on the set goal quite well. 

Cost matrix choosen is 
`Q = np.diag([1, 1, 10, 100]);`
These values are chosen by analizing the sensitivity of each state variable towards keeping the system in the linearized window.
Angular position of pole needs to be maintained within a narrow bound and angular velocity of pole needs to be maintained low to achieve the first.
Hence the choice of state cost matrix. 

The R matrix has the same number of rows as are control inputs and the same number of columns as are control inputs.
The input cost matrix R often has positive values along the diagonal. We can use this matrix to target actuator states where we want low actuator effort by making the corresponding value of R large.

Since controllability of inverted pendulum is of higher priority than conserving actuator effort, a value lower than 1 is chosen.
`R = np.diag([0.1])`

