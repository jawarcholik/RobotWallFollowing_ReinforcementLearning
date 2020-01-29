#!/usr/bin/python3

#ros imports
import rospy
from geometry_msgs.msg import Pose2D

#std imports
from threading import Thread
import time
from pynput import keyboard


#establish ros node and publisher to velocity
rospy.init_node("teleop_robot")
vel_pub = rospy.Publisher("/triton/vel_cmd", Pose2D, queue_size=2)

LIN_SPEED = 0.2
ANG_SPEED = 1.0

vel_msg = Pose2D()
key_state = {}
def key_update(key, state):

    #key is pressed for the first time
    if key not in key_state:
        key_state[key] = state
        return True

    # key changed state
    if state != key_state[key]:
        key_state[key] = state
        return True

    #no change
    return False



def key_press(key):
    try:
        #character input
        k = key.char
    except:
        #arrow key/other input
        k = key.name

    #check if press changes state
    change = key_update(key, True)
    if change:
        global vel_msg
        if   k in ['w', 'up']:
            vel_msg.y += LIN_SPEED
        elif k in ['s', 'down']:
            vel_msg.y -= LIN_SPEED
        elif k in ['d', 'right']:
            vel_msg.x += LIN_SPEED
        elif k in ['a', 'left']:
            vel_msg.x -= LIN_SPEED
        elif k in ['e']:
            vel_msg.theta += ANG_SPEED
        elif k in ['q']:
            vel_msg.theta -= ANG_SPEED
    
        vel_pub.publish(vel_msg)
    

def key_release(key):
    try:
        #character input
        k = key.char
    except:
        #arrow key/other input
        k = key.name

    change = key_update(key, False)
    if change:
        global vel_msg
        if   k in ['w', 'up']:
            vel_msg.y += -LIN_SPEED
        elif k in ['s', 'down']:
            vel_msg.y -= -LIN_SPEED
        elif k in ['d', 'right']:
            vel_msg.x += -LIN_SPEED
        elif k in ['a', 'left']:
            vel_msg.x -= -LIN_SPEED
        elif k in ['e']:
            vel_msg.theta += -ANG_SPEED
        elif k in ['q']:
            vel_msg.theta -= -ANG_SPEED
    
        vel_pub.publish(vel_msg)

def user_display():
    print('Use WSAD or the ARROW KEYS to control Triton. Use Q & E to rotate Triton. ')
    while True:
        try:
            print("\rX: {}\tY: {}\tTHETA: {}          ".format(vel_msg.x, vel_msg.y, vel_msg.theta)),
            time.sleep(0.1)
        except KeyboardInterrupt:
            return


#start key listener thread
key_listener = keyboard.Listener(on_press=key_press, on_release=key_release) 
key_listener.start()

#start user display thread
display_thread = Thread(target=user_display)
display_thread.start()

#update ros topics on main thread
rospy.spin()

