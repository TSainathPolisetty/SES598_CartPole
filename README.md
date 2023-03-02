# SES598_CartPole
Inverted Pendulum LQR control

## Launch the gazebo model using

`roslaunch invpend_control load_invpend.launch`

## PID
### PID controller parameter tuning

Operate rqt_reconfigure to modify PID values launch rqt_reconfigure with the following command

`rosrun rqt_reconfigure rqt_reconfigure`

### Demo: Inbuilt PID controller

Demo includes setting pole to horizontal and publishing velocity commands. 
