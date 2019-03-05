#!/usr/bin/env python
# license removed for brevity

from static_phycal import *
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray 
from rospy.numpy_msg import numpy_msg
import time

#suT = 0

joyK = 0
joy_button_data = [0, 0, 0]
joy_left_data = [0, 0]
forwardFlag = 0
state_data = 0
autoSwitch = 0
Kp = 0

def trigger_cb(gdata):
    global Kp
    if state_data == 0 or state_data == -1:
        pub_data = [0 for i in range(8)]
        pub_data = Float32MultiArray(data = pub_data)
        pub1.publish(pub_data)
        pub2.publish(0)
        return
        
    #test_mat_all = np.matrix([[forward_data], [0], [0], [0], [0], [0]])
    if autoSwitch%2 == 1:
        Foutrate = joyK
    else:
        Foutrate = forwardFlag
    result_F = Tax*Kp*Foutrate
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)
    pub2.publish(Kp*Foutrate)

def state_cb(data):
    global state_data
    state_data = data.data

def turnflag_cb(data):
    global forwardFlag
    data = data.data
    if data[2] == 0 and data[3] == 0:
        forwardFlag = data[2]
    else:
        forwardFlag = 1

def joyB_cb(data):
    global joy_button_data, autoSwitch
    joy_button_data = data.data
    front_sig = joy_button_data[2]
    if (front_sig >> 3)%2 == 1:
        autoSwitch = (autoSwitch+1)%2
        pub3.publish(autoSwitch)
        
def joyL_cb(data):
    global joy_left_data, joyK
    joy_left_data = data.data
    y_sig = joy_left_data[1]
    if y_sig > 0:
        joyK = 1
    elif y_sig < 0:
        joyK = -1
    else:
        joyK = 0
'''
def time_cb(data):
    data = data.data
    if data > 9:
        ax = 0.1
    else:
        ax = 0
    result_F = Tax*ax
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)
'''

#F = 6.23N = 1.4lbf = pwm1600
Kp = 1
#Kp = (Tax[2]+Tax[3])/(2*6.23)

rospy.init_node('forwardPID',anonymous=True)

rospy.Subscriber('/trigger_command', Int32, trigger_cb)
rospy.Subscriber('/state', Int32, state_cb)
rospy.Subscriber('/flag/PIDturn', Int32MultiArray, turnflag_cb)
rospy.Subscriber('/joy/button', Int32MultiArray, joyB_cb)
rospy.Subscriber('/joy/left', Int32MultiArray, joyL_cb)
#rospy.Subscriber('sumi_t', Float32, time_cb)

pub1 = rospy.Publisher('/force/forward',Float32MultiArray,queue_size=10)
pub2 = rospy.Publisher('/ft/forward',Float32,queue_size=10)
pub3 = rospy.Publisher('/joy/flag/forward',Int32,queue_size=10)

while not rospy.is_shutdown():
    pass


