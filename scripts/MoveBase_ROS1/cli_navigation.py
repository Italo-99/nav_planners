#!/usr/bin/env python3
from re import sub
import rospy
from geometry_msgs.msg import PointStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
import os
import sys
import csv
import subprocess

points_list = []
pub = rospy.Publisher('/waypoints', PointStamped, queue_size=10)
rospy.init_node('cli_navigation', anonymous=True)

client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

# WayPoints list
QX = 12.75
QY = 3.12

BX = 25.38
BY = 3.84

WHX = 16.58
WHY = -14.56

FX = 10.61
FY = -19.07

HX = 0.0
HY = 0.0

#Waypoints Preparation Zone
#Globals
AX = -1.17
AY = 13.66

CX = 8.37
CY = 12.64

DX = 4.36
DY = 6.12

#Locals
EX = 2.0
EY = 0.0

FX = 1.0
FY = -3.0

GX = 2.0
GY = -4.0


# Show a list of waypoints
def show_wp():
    print("\n\n")
    print("=======================================================")
    print("===== Project RED Autonomous Navigation interface =====")
    print("=======================================================")
    print("")
    print('The WayPoint list is:')
    print("Waypoint Q (auto):       x:%f      y:%f" %(QX,QY))
    print("Waypoint B (manual):     x:%f      y:%f" %(BX,BY))
    print("Waypoint H (manual):     x:%f      y:%f" %(WHX,WHY))
    print("Waypoint F (auto):     x:%f      y:%f" %(FX,FY))
    print("Home (manual):     x:%f      y:%f" %(HX,HY))
    print("")
    print("Preparation Zone Waypoints:")
    print("Global coordinates:")
    print("Waypoint A:       x:%f      y:%f" %(AX,AY))
    print("Waypoint C:       x:%f      y:%f" %(CX,CY))
    print("Waypoint D:        x:%f      y:%f" %(DX,DY))
    print("Local coordinates:")
    print("Waypoint E:       x:%f      y:%f" %(EX,EY))
    print("Waypoint F:       x:%f      y:%f" %(FX,FY))
    print("Waypoint G:       x:%f      y:%f\n" %(GX,GY))


# FUNZIONE PER SCEGLIERE IL WAYPOINT DA RAGGIUNGERE
def switch(i):
    switcher= {
        'Q': [QX,QY],
        'q': [QX,QY],
        'B': [BX,BY],
        'b': [BX,BY],
        'H': [WHX,WHY],
        'h': [WHX,WHY],
        'F': [FX,FY],
        'f': [FX,FY],
        'Home': [HX,HY],
        'home': [HX,HY],
        }
    return switcher.get(i)           # WQ = default




def movebase_client(x,y):

    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal, done_cb=callback_done)

def callback_active():
    print("Action Server Processing the Goal")

def callback_feedback(feedback):
    print("Feedback: {}".format(feedback))

def callback_done(state, result):
    print("Action server is done")
    print("State: {}".format(state))
    #print("Result: {}".format(result))

def stop():
    client.cancel_goal()
    
if __name__ == '__main__':
    
    while True:
        # Show list of waypoints to choose from
        show_wp()

        # Get input 
        num = input("Select WayPoint to reach (type 'exit' to leave):")
        if (num == 'stop'):
            stop()
        elif(num == 'exit'):
            break
        else:
            [wp_x, wp_y] = switch(num)
            movebase_client(wp_x, wp_y)

        #if result:
           # rospy.loginfo("Goal execution done!")