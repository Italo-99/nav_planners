#!/usr/bin/env python3

#VISUALIZATION OF RACE WAYPOINTS

#import libraries
import numpy as np
import pandas as pd
import math as m
import time as tm
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

markerArray = MarkerArray()

def read_WPScsv():

    rospy.loginfo("Read the csv to get waypoints written on the competition document")
    df = pd.read_csv("src/navigation/script/csv_tables/WaypointsGara.csv", header=None)
    WPS = np.array(df)

    return WPS

def nameWps(i):

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


def pub_wp(i,WPS):

    rospy.loginfo("Publishing waypoint [%f,%f]", WPS[i-1][0], WPS[i-1][1])

    marker = Marker()

    marker.header.frame_id = "world"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.ns = nameWps(i)
    marker.id = i
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = WPS[i-1][0]
    marker.pose.position.y = WPS[i-1][1]
    marker.pose.position.z = 0

    markerArray.markers.append(marker)

def main():

    topic = 'visualization_RaceWaypoints'
    WPS_pub = rospy.Publisher(topic, MarkerArray, queue_size = 40)
    rospy.init_node('WPS_visualizer', anonymous=True)

    rospy.loginfo("Publishing waypoints")

    WPS = read_WPScsv()

    rospy.loginfo("Found %i waypoints in the file", len(WPS))

    for i in range(len(WPS)):
        
            pub_wp(i+1,WPS)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        WPS_pub.publish(markerArray)

        rate.sleep()


if __name__ == '__main__':
    main()