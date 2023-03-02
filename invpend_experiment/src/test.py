#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
import control as ct
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point
import math

m1 = 20;
m2 = 2;
l = 0.5;
I = m1*l*l/12;#0.05


# ~ #====================
# ~ # Uniform mass rod
# ~ den = (4*m1+m1);

# ~ A23 = 3*m2*g/den;
# ~ A43 = -6*(m1+m2)*g/(den*l);

# ~ B2 = 4/den;
# ~ B4 = -6/(den*l);

# ~ #===================
# ~ # Rod with m and I
den = m1*m2*l*l + 4*m1*I + 4*m2*I;

A23 = -m2*m2*l*l*g/den;
A43 = 2*l*m2*(g*m1+g*m2)/den;

B2 = (4*I+m2*l*l)/den;
B4 = -2*l*m2/den;

A = np.matrix([[0,1,0,0],
       [0, 0, A23, 0],
       [0, 0, 0, 1],
       [0, 0, A43, 0]
       ])
B = np.matrix([0, B2, 0, B4]).T

C = np.matrix([[1,0,0,0], [0,0,1,0]])
D = np.matrix([0,0]).T

Q = np.diag([1, 1, 10, 100])
R = np.diag([0.1])

############## Finding the State feedback gains ######################
K, S, E = lqr( A, B, Q, R )
############Postion & Angle of Cart #######################################
cart_pose =np.array([0., 0., 0., 0.])  # General Format[x,x_dot,Θ,Θ_dot]
final_pose = np.array([0., 0., 0., 0.])


####################Balcing the Ploe using LQR , u=-Kx ######################
def pole_blanace():
    rospy.init_node('p_blanace')
    print("Cart Pole Pose & Angle",cart_pose)
    vel_publisher = rospy.Publisher("/invpend/joint1_velocity_controller/command", Float64, queue_size=50)
    vel_msg  = Float64()
    x=cart_pose-final_pose
    vel_msg.data = -np.matmul(K,x)     #u =-Kx
    vel_publisher.publish(vel_msg)
    
##### Callback to update the theta and theta_dot values of Pole     
def takeangle(angle_message):
    cart_pose[2] = angle_message.process_value
    cart_pose[3] = angle_message.process_value_dot
    
##### Callback to update the Postion and Velocity values of Cart Pole     
def takepose(pose_message):
    cart_pose[0] = pose_message.position[1]
    cart_pose[1] = pose_message.velocity[1]
    
    
if __name__ == '__main__':
    theta_sub = rospy.Subscriber("/invpend/joint2_position_controller/state",JCS, takeangle)
    pos_sub = rospy.Subscriber("/invpend/joint_states",JS, takepose) 
    while not rospy.is_shutdown():
        pole_blanace()
    def check_controllability(self):
        ss = ct.ss(A,B,C,D)
        _ctrb = ct.ctrb(ss.A, ss.B)
        _rank = np.linalg.matrix_rank(_ctrb)
        print("The rank of the system is ",_rank)
        if _rank == 4:
            print("Therefore, it is a Controllable System")

if __name__ == '__main__':
    print("hello")
    rospy.init_node('CartController', anonymous=True)
    cartpole = CartPoleLQR()
    cartpole.check_controllability()
    cartpole.controlLoop()
    rospy.spin()
