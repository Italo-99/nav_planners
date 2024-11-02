"""#SETUP CODE -> GRAPH GENERATION (uncomment to overwrite previous waypoints planner)

# #MODEL PARAMETERS 
df = pd.read_csv("src/navigation/script/csv_tables/WaypointsPlanner.csv", header=None, names=['X_wpr','Y_wpr'])
V = np.array(df)            #array of roadmap waypoints, vertices of the graph
 
N = len(V)                  #number of waypoints of the roadmap
P = np.zeros((N,N))         #init distances waypoints matrix (costs/weights of the graph)
E = np.zeros((N,N))         #init arcs between waypoints matrix (links in thegraph)

# #Compute the matrix of distances and of arcs
for i in range(N):
    for j in range(i+1,N):
        P[i][j] = distance(V[i],V[j])    #compute the distance between wp[i] and wp[j]
        if  P[i][j] > 6:
            P[i][j] = 1000  #if two waypoints are at distance greater than 6, they are not connected in the roadmap
            P[j][i] = 1000  
            E[i][j] = 0     #there is not a arc between them
            E[j][i] = 0
        else:
            E[i][j] = 1     #if their relative distance is less than 6 m, they are linked by an arc in the graph
            E[j][i] = 1
            P[j][i] = P[i][j]

# #INPUT OF THE CODE -> LIST OF WAYPOINTS TO REACH
WP_array = pd.read_csv("src/navigation/script/csv_tables/WaypointsGara.csv", header=None, names=['X_wp','Y_wp'])
WP_array = np.array(WP_array)   #array of competiotion waypoints, input of the graph
n = len(WP_array)               #number of waypoints of the competition

# #FIRST PART (OFFLINE) -> COMPUTE THE MINIMUM PATH BETWEEN EACH COUPLE OF WAYPOINTS

init_setup = tm.perf_counter()

dist = np.zeros((N,N))+m.inf    #matrix of distances of each waypoint from all the other waypoints: init as inf in all positions outside main diagonal
for k in range(N):
    dist[k][k] = 0              #distances 0 of each waypoint with himself

Q = np.zeros((N,N))             #vector to store if a Waypoint has been visited

for i in range(N):
    while np.min(Q[i])<1:
        min, ind_min = mindist_Q(Q[i],dist[i])
        Q[i][ind_min] = 1
        for j in range(N):
            if E[ind_min][j] == 1 and ind_min!=j:
                if dist[i][j] > dist[i][ind_min]+P[ind_min][j]:
                    dist[i][j] = dist[i][ind_min]+P[ind_min][j]

end_setup = tm.perf_counter()
print(end_setup-init_setup)

df = pd.DataFrame(dist)
index = []
for k in range(N):
    index.append(str(k+1))
print(df)

#df.to_csv(r'C:\Users\Italo\Desktop\Progetti\ProjectRed\Robotica\Localization\Waypoints_MinDistances.csv', index = index)
df.to_csv('src/navigation/script/csv_tables/WaypointsGara.csv', index = None, header=None)"""