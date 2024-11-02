#!/usr/bin/env python3

#PATH GENERATION - OPTIMIZATION BETWEEN PRE-BUILD WAYPOINTS WITH THE ALGORITHM OF DJIKSTRA

#import libraries
import numpy as np
import pandas as pd
import math as m
import time as tm
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PointStamped, Transform, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import actionlib, actionlib_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#function: compute distances between waypoints
def distance(wp_1,wp_2):
    return(m.sqrt(pow(wp_1[0]-wp_2[0],2)+pow(wp_1[1]-wp_2[1],2)))

#definition of a function to compute the minimum of a certain group of an array
def mindist_Q(Q,dist):
    min = m.inf
    min_ind = len(dist)+1
    for k in range(len(dist)):
        if dist[k]<min and Q[k]==0:
            min = dist[k]
            min_ind = k
    return min, min_ind

#SECOND PART (OFFLINE) -> SEARCH THE MINIMUM PATH BETWEEN START AND GOAL

def minPath(wp_i,wp_f):

    #User can choose start and goal among those in the file "WaypointsGara" (they are passed to this function as integers in [1,30])
    ind_s = wp_i-1
    ind_f = wp_f-1

    WP_array = pd.read_csv("src/navigation/script/csv_tables/WaypointsGara.csv", header=None, names=['X_wp','Y_wp'])
    WP_array = np.array(WP_array)  
    n = len(WP_array)
    df = pd.read_csv("src/navigation/script/csv_tables/WaypointsPlanner.csv", header=None, names=['X_wpr','Y_wpr']) 
    WP_planner = np.array(df)  
    N = len(WP_planner) 
    df = pd.read_csv("src/navigation/script/csv_tables/Waypoints_MinDistances.csv", header=None)
    Min_dist = np.array(df)

    #Search the entrance of the roadmap as the shortest distance among WP_planner waypoints
    dist_s = np.zeros((N))
    dist_f = np.zeros((N))
    for k in range(N):
        dist_s[k]=distance(WP_array[ind_s],WP_planner[k])
        dist_f[k]=distance(WP_array[ind_f],WP_planner[k])
    ind_RS = np.argmin(dist_s)
    ind_RF = np.argmin(dist_f)

    #Search in Min_dist matrix the row corresponding to the minimum distances of paths starting from point selected as ind_s
    dist = Min_dist[ind_RS]

    #Minimum path to search
    Min_path_length = dist[ind_RF]
    print("Min_path_length: ", Min_path_length)

    #definition of the connectivity graph (REVERSED): if the distance between two wps is more than 6 metres, than they are not considered as connected 
    E = np.zeros((N,N))             
    for i in range(N):
        for j in range(N):
            if distance(WP_planner[i],WP_planner[j]) > 6 :
                E[i][j] = 1
            else :
                E[i][j] = 0

    #Algorithm to search the minimum path
    path_length = Min_path_length
    ind_current_node = ind_RF                #start to search the path from the goal point
    path_seq = [ind_RF]                      #sequence of the indeces of the waypoints the path goes through

    while path_length > 0:
        min, ind_min = mindist_Q(E[ind_current_node],dist)
        path_length = min
        ind_current_node = ind_min
        path_seq.append(ind_current_node)

    """
    #Print the path
    print(ind_f)
    print(path_seq)
    print(ind_s)

    print(WP_array[ind_s])
    for k in range(len(path_seq)):
        print(WP_planner[path_seq[len(path_seq)-k-1]])
    print(WP_array[ind_f])
    """

    #path_seq.insert(0,ind_s)
    #path_seq.append(ind_f)

    return path_seq

def callback_done(state, result, feedback):
    print("Action server is done")
    print("State: {}".format(state))
    print("Result: {}".format(result))
    print("Feedback: {}".format(feedback))

def movebase_client(x,y,mb_client):

    mb_client.wait_for_server()

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

    mb_client.send_goal(goal, done_cb=callback_done, active_cb = None, feedback_cb = None)

def main():

    """
    #move base client
    mb_pub = rospy.Publisher('/waypoints', PointStamped, queue_size=10)
    rospy.init_node('cli_navigation', anonymous=True)
    mb_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    """

    #Path generation visualizer
    rospy.init_node("Djikstra_pathGenration")
    path_publisher = rospy.Publisher("PathDjikstra_Visualizer", Path, queue_size=1)

    WP_array = pd.read_csv("src/navigation/script/csv_tables/WaypointsGara.csv", header=None, names=['X_wp','Y_wp'])
    WP_array = np.array(WP_array)  
    df = pd.read_csv("src/navigation/script/csv_tables/WaypointsPlanner.csv", header=None, names=['X_wpr','Y_wpr']) 
    WP_planner = np.array(df)  

    #rate = rospy.Rate(10)

    while True:

        # Get input from the user
        wp_i = input("Select starting waypoint (type 'exit' to leave):")
        if (wp_i == 'exit'):
            break
        else:
            wp_f = input("Select final waypoint to generate a path (type 'exit' to leave):")

        wp_i = int(wp_i)
        wp_f = int(wp_f)

        if not(wp_f <= 30 and wp_f >= 1 and wp_i <= 30 and wp_i >= 1):

            rospy.loginfo("Error while choosing starting and goal waypoints")

        else:
            
            init_time = tm.perf_counter()

            WP_Path = minPath(wp_i,wp_f)

            msg = Path()
            msg.header.frame_id = "world"
            msg.header.stamp = rospy.Time.now()

            """
            print(WP_array[ind_s])
            for k in range(len(path_seq)):
                print(WP_planner[path_seq[len(path_seq)-k-1]])
            print(WP_array[ind_f])
            """

            for k in range(len(WP_Path)):
                
                pose = PoseStamped()

                pose.pose.position.z = 0
                pose.pose.orientation.x = 0
                pose.pose.orientation.y = 0
                pose.pose.orientation.z = 0
                pose.pose.orientation.w = 1

                """
                if k == 0:
                    pose.pose.position.x = WP_array[wp_i][0]
                    pose.pose.position.y = WP_array[wp_i][1]

                    rospy.loginfo("WP %i [%f,%f]", wp_i, WP_array[wp_i][0], WP_array[wp_i][1])
                    
                if k == len(WP_Path)-1:
                    pose.pose.position.x = WP_array[wp_f][0]
                    pose.pose.position.y = WP_array[wp_f][1]               
                    
                    rospy.loginfo("WP %i [%f,%f]", wp_f, WP_array[wp_f][0], WP_array[wp_f][1])
                else :
                    pose.pose.position.x = WP_planner[WP_Path[len(WP_Path)-k-1]][0]
                    pose.pose.position.y = WP_planner[WP_Path[len(WP_Path)-k-1]][1]

                    rospy.loginfo("WP %i [%f,%f]", WP_Path[len(WP_Path)-k-1], WP_planner[WP_Path[len(WP_Path)-k-1]][0], WP_planner[WP_Path[len(WP_Path)-k-1]][1])
                """

                pose.pose.position.x = WP_planner[WP_Path[len(WP_Path)-k-1]][0]
                pose.pose.position.y = WP_planner[WP_Path[len(WP_Path)-k-1]][1]

                rospy.loginfo("WP %i [%f,%f]", WP_Path[len(WP_Path)-k-1], WP_planner[WP_Path[len(WP_Path)-k-1]][0], WP_planner[WP_Path[len(WP_Path)-k-1]][1])

                msg.poses.append(pose)                

                #movebase_client(WP_planner[WP_Path[len(WP_Path)-k-1]][0], WP_planner[WP_Path[len(WP_Path)-k-1]][1])

            rospy.loginfo("Computed path in %f seconds ", tm.perf_counter() - init_time )

            rate = rospy.Rate(10)

            #NON RIESCO A PULIRE IL TRAGITTO E A RISCRIVERLO SU RVIZ

            while not rospy.is_shutdown():

                path_publisher.publish(msg)

                rate.sleep()
             
        #rate.sleep()

if __name__ == '__main__':
    main()