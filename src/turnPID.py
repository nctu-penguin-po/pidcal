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

joy_button_data = [0, 0, 0]
joy_right_data = [0, 0]
turn_kp = 1
turn_flag = [0, 0] #left, right
state_data = 0
autoSwitch = 0
joyK = 0
F_value = 0
F_corK = 1

def trigger_cb(data):
    global turn_kp, state_data, joyK, autoSwitch, F_corK, F_value
    turn=data.data
    print(state_data)
    if (state_data >> 3)%2 == 0 or state_data == -1:
        turn_direction = 0
        pub2.publish(0)
        pub_data = [0 for i in range(8)]
        pub_data = Float32MultiArray(data = pub_data)
        pub1.publish(pub_data)
        return
    if autoSwitch%2 == 1:
        turn_direction = joyK
    elif turn_flag[0] > 0:
        turn_direction = 1
    elif turn_flag[1] > 0:
        turn_direction = -1
    else:
        turn_direction = 0
    turn_direction = turn_direction-F_value*F_corK 
    result_F = Talphaz*turn_kp*turn_direction
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)
    pub2.publish(turn_kp*turn_direction)

def state_cb(data):
    global state_data, autoSwitch, joyK
    state_data = data.data
    if (state_data >> 3)%2 == 1:
        autoSwitch = 1
        joyK = -5
    elif (state_data >> 3)%2 == 1:
        autoSwitch = 1
        joyK = 5
    else:
        autoSwitch = 0
    pub3.publish(autoSwitch)

def turnKp_cb(data):
    global turn_kp
    data = data.data
    rowkp = data

def turnflag_cb(data):
    global turn_flag
    data = data.data
    turn_flag[0] = data[0]
    turn_flag[1] = data[1]

def joyB_cb(data):
    global joy_button_data, autoSwitch
    joy_button_data = data.data
    front_sig = joy_button_data[2]
    if front_sig%2 == 1:
        autoSwitch = (autoSwitch+1)%2
        pub3.publish(autoSwitch)
        
def joyR_cb(data):
    global joy_right_data, joyK
    joy_right_data = data.data
    x_sig = joy_right_data[0]
    if x_sig > 0:
        joyK = -5
    elif x_sig < 0:
        joyK = 5
    else:
        joyK = 0

def getForward_cb(data):
    global F_value
    F_value = data.data

'''
def time_cb(data):
    data = data.data
    result_F = Talphaz*0
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)
'''

rospy.init_node('turnPID',anonymous=True)

rospy.Subscriber('/trigger_command', Int32, trigger_cb)
rospy.Subscriber('/state', Int32, state_cb)
rospy.Subscriber('/PIDpara/turn', Float32, turnKp_cb)
rospy.Subscriber('/flag/PIDturn', Int32MultiArray, turnflag_cb)
rospy.Subscriber('/joy/button', Int32MultiArray, joyB_cb)
rospy.Subscriber('/joy/right', Int32MultiArray, joyR_cb)
rospy.Subscriber('/ft/forward', Float32, getForward_cb)
#rospy.Subscriber('/sumi_t', Float32, time_cb)

pub1 = rospy.Publisher('/force/turn',Float32MultiArray,queue_size=10)
pub2 = rospy.Publisher('/ft/turn',Float32,queue_size=10)
pub3 = rospy.Publisher('/joy/flag/turn',Int32,queue_size=10)

while not rospy.is_shutdown():
    pass


