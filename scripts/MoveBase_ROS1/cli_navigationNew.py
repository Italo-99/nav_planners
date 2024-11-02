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

import numpy as np
import pandas as pd

points_list = []
pub = rospy.Publisher('/waypoints', PointStamped, queue_size=10)
rospy.init_node('cli_navigation', anonymous=True)

client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

def nameWps(i):

    s = "null"

    if i == 1:
        s = "C" #staring point
    #navigation waypoints
    elif i == 2:
        s= "WA"
    elif i == 3:
        s= "WB"
    elif i == 4:
        s= "WC"
    elif i == 5:
        s= "WF"
    elif i == 6:
        s= "WG"
    elif i == 7:
        s= "WH"
    elif i == 8:
        s= "WL"
    elif i == 9:
        s= "WQ"    
    #probing starting point
    elif i == 10:
        s= "C0"
    #probing waypoints
    elif i == 11:
        s= "PK"
    elif i == 12:
        s= "PM"
    elif i == 13:
        s= "PN"
    elif i == 14:
        s= "PP"
    elif i == 15:
        s= "PT"
    elif i == 16:
        s= "PX"
    elif i == 17:
        s= "PZ"
    #collection waypoints
    elif i == 18:
        s= "C1"
    elif i == 19:
        s= "C2"
    elif i == 20:
        s= "C3"
    elif i == 21:
        s= "C4"
    elif i == 22:
        s= "C5"
    elif i == 23:
        s= "C6"
    elif i == 24:
        s= "C7"
    elif i == 25:
        s= "C8"
    elif i == 26:
        s= "C9"
    elif i == 27:
        s= "C10"
    elif i == 28:
        s= "C11"
    elif i == 29:
        s= "C12"
    elif i == 30:
        s= "D1"    
     
    return s

def read_WPScsv():

    rospy.loginfo("Read the csv to get waypoints written on the competition document")
    df = pd.read_csv("src/navigation/script/csv_tables/WaypointsGara.csv", header=None)
    WPS = np.array(df)

    return WPS

# Show a list of waypoints
def show_wp():
    print("\n\n")
    print("=======================================================")
    print("===== Project RED Autonomous Navigation interface =====")
    print("=======================================================")
    print("")
    print('The WayPoint list is:')

    WPS = read_WPScsv()

    for k in range(len(WPS)):
        print("%i-th Waypoint %s: x:%f      y:%f" %(k+1,nameWps(k+1),WPS[k,0],WPS[k,1]))

def movebase_client(x,y):

    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "world"
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
            WPS=read_WPScsv()
            movebase_client(WPS[int(num)-1,0], WPS[int(num)-1,1])

        #if result:
           # rospy.loginfo("Goal execution done!")