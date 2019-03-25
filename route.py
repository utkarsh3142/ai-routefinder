#!/bin/python

import heapq
import sys
import pandas as pd
import math

#Store city-gps data using pandas dataframe
city_columns = ['city','latitude','longitude']
city_df = pd.read_csv('city-gps.txt', delim_whitespace=True,header=None, names=city_columns)

#Store road-segments data using pandas dataframe
roads_columns = ['city1','city2','distance','speed_limit','highway_name' ]
roads_df = pd.read_csv('road-segments.txt', delim_whitespace=True,header=None, names=roads_columns)

#Calculate time based on distance and speed limit and add time as a column to road segments dataframe
roads_df['speed_limit'] = pd.to_numeric(roads_df['speed_limit'], errors='coerce').fillna(0)
roads_df['time'] = roads_df['distance']/roads_df['speed_limit']

#Get list of all the nodes connected by road segments to a particulate node(state)
def successors(state):
     return roads_df.loc[roads_df['city1'] ==state][['city2', 'distance', 'time','highway_name']].values.tolist() + roads_df.loc[roads_df['city2'] == state][['city1', 'distance', 'time','highway_name']].values.tolist()

#Check if given node is goal
def is_goal(state):
     return state == goal

#Solver function for Iterative Deeping Depth First Search
def solve_ids(initial_state):
    depth_so_far = {}
    time_so_far = {}
    dist_so_far = {}

    for k in range(0,city_df.shape[0]):
        fringe = [(initial_state,"")]
        time_so_far[initial_state] = 0
        dist_so_far[initial_state] = 0
        depth_so_far[initial_state] = 0
        while len(fringe) > 0:
                (state,route_so_far) = fringe.pop()
                if depth_so_far[state] < k:
                        for [succ, dist, time, highway] in successors( state ):
                                if is_goal(succ):
                                        return  (dist_so_far[state] + dist,time_so_far[state]+ float(time),initial_state +  route_so_far + " " + succ)
                                cost = time_so_far[state] + float(time)
                                curr_dist = dist_so_far[state] + dist
                                depth_so_far[succ] = depth_so_far[state]+1
                                if succ not in dist_so_far or dist < dist_so_far[succ]:
                                        dist_so_far[succ] = curr_dist
                                if succ not in time_so_far or cost < time_so_far[succ]:
                                        time_so_far[succ] = cost
                                if succ != initial_state:
                                        data = (succ,route_so_far + " " + succ)
                                        fringe.append(data)
    return False




#Solver function for Breadth First Search and Depth First Search
def solve_bd(initial_state,q):
    fringe = [(initial_state,"")]
    time_so_far = {}
    dist_so_far = {}
    time_so_far[initial_state] = 0
    dist_so_far[initial_state] = 0
    while len(fringe) > 0:
       (state,route_so_far) = fringe.pop(q)
       for [succ, dist, time, highway] in successors( state ):
            if is_goal(succ):
                return  (dist_so_far[state] + dist,time_so_far[state]+ float(time),initial_state + " " + route_so_far  + " " + succ)
            cost = time_so_far[state] + float(time)
            curr_dist = dist_so_far[state] + dist
            if succ not in dist_so_far or dist < dist_so_far[succ]:
                dist_so_far[succ] = curr_dist
            if succ not in time_so_far or cost < time_so_far[succ]:
                time_so_far[succ] = cost
            data = (succ,route_so_far + " " + succ)
            fringe.append(data)

    return False


#Heuristics function for astar search
def heuristics(cost_type,dest,succ):
#Check if succ is in cities list or not. If it is a junction or not in city list, below code will get the closest city to succ and calculate heuristics based on that neighbour city distance
    cities = city_df['city'].values.tolist()
    if succ not in cities:
        neighbours = roads_df[roads_df['city1'] == 'Jct_I-465_&_IN_37_S,_Indiana'][['city2','distance']].values.tolist() + roads_df[roads_df['city2'] == 'Jct_I-465_&_IN_37_S,_Indiana'][['city1','distance']].values.tolist()
        neighbour_set = {}
        for c_name, c_dist in neighbours:
                if c_name in city_df['city'].values.tolist():
                        neighbour_set[c_name] = c_dist
        succ = min(neighbour_set, key=neighbour_set.get)
    [[lat1,lon1]] = city_df[city_df['city']==dest][['latitude','longitude']].values.tolist()
    [[lat2,lon2]] = city_df[city_df['city']==succ][['latitude','longitude']].values.tolist()
#This is calculated using haversine https://en.wikipedia.org/wiki/Haversine_formula and the code below for calculating is based on https://janakiev.com/blog/gps-points-distance-python/ blog on calculating distance using this formula
    radius = 3959 #6371
    dlat = math.radians(lat2-lat1)
    dlon = math.radians(lon2-lon1)
    a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(math.radians(lat1))* math.cos(math.radians(lat2)) * math.sin(dlon/2) * math.sin(dlon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    #c = 2 * math.asin(math.sqrt(a))
    d = radius * c
#Return heuristics based on type, for time, we have assumed the average speed to be 60. For distance and segment, it will return the values of distance from succ to goal node
    if cost_type == "distance":
        return d#*0.621371              #return distance in miles
    elif cost_type == "time":
        return d/60#*0.621371/60
    elif cost_type == "segments":
        return d#*0.621371


#Solver function for Uniform Cost Search and Astar Search
def solve_ua(initial_state,algorithm,cost_type):

#Initialise fringe as a heapq and push initial_state with priority 0
    fringe = []
    heapq.heappush(fringe,(0,(initial_state, "")))
#Below dictionaries will store the total distance, total time and total number of nodes(segments) corresponding to a given node
    depth_so_far = {}
    time_so_far = {}
    dist_so_far = {}
#Initial node will have total distance, total time, total number of nodes(segments) as 0
    depth_so_far[initial_state] = 0
    time_so_far[initial_state] = 0
    dist_so_far[initial_state] = 0

    while len(fringe) > 0:
        (p,(state, route_so_far)) = heapq.heappop(fringe)
        for [succ,dist,time, highway] in successors( state):
            if is_goal(succ):
                return( dist_so_far[state] + dist,round(time_so_far[state]+ float(time),3),initial_state + " " + route_so_far + " " + succ)
#Calculate current time,distance and depth
            time_cost = time_so_far[state] + float(time)
            dist_cost = dist_so_far[state] + dist
            depth_cost = depth_so_far[state] + 1
#If problem is astar, then calculate heuristics, else h will be 0
            h_cost = heuristics(cost_type,goal,succ) if algorithm == "astar" else 0
#Based on the cost type, calculate f = g + h(0 in case of Uniform Cost Search) and push succ with priority as f
            if succ != initial_state:
                if succ not in depth_so_far or depth_cost < depth_so_far[succ]:
                        depth_so_far[succ] = depth_cost
                        if cost_type == "segments": heapq.heappush(fringe,(depth_cost+h_cost,(succ,route_so_far + " " + succ)))
                if succ not in dist_so_far or dist_cost < dist_so_far[succ]:
                        dist_so_far[succ] = dist_cost
                        if cost_type == "distance": heapq.heappush(fringe,(dist_cost+h_cost,(succ,route_so_far + " " + succ)))
                if succ not in time_so_far or time_cost < time_so_far[succ]:
                        time_so_far[succ] = time_cost
                        if cost_type == "time": heapq.heappush(fringe,(time_cost+h_cost,(succ,route_so_far + " " + succ)))
    return False


#Accept command line arguments as: 1.Start City 2.End City 3.Algorithm Type 4.Cost Type
if len(sys.argv) == 5:
     initial_state = sys.argv[1]
     goal = sys.argv[2]
     algorithm = sys.argv[3]
     cost_type = sys.argv[4]
elif len(sys.argv) == 4:
     initial_state = sys.argv[1]
     goal = sys.argv[2]
     algorithm = sys.argv[3]
     cost_type = 'segments'
elif len(sys.argv) == 3:
     initial_state = sys.argv[1]
     goal = sys.argv[2]
     print "Taking BFS as default algorithm"
     algorithm = 'bfs'
     cost_type = "segments"
else:
     print "Incorrect Arguments"

optimal = ""
#Select solver based on Algorithm Type
if algorithm == 'bfs':
     solution = solve_bd(initial_state,0)
     optimal = 'yes' if cost_type == 'segments' else 'no'
elif algorithm == 'dfs':
     solution = solve_bd(initial_state,-1)
     optimal = "no"
elif algorithm == 'astar' or algorithm == 'uniform':
     if cost_type in ['segments','distance','time']:
        solution = solve_ua(initial_state,algorithm,cost_type)
        optimal = 'no' if algorithm == 'astar' else 'yes'
     else:
        print "Taking distance as default cost type"
        solution = solve_ua(initial_state,algorithm,'distance')
        optimal = 'no' if algorithm == 'astar' else 'yes'
elif algorithm == 'ids':
     solution = solve_ids(initial_state)
     optimal = 'yes'
else:
     print "Taking BFS as default algorithm and cost_type as segment"
     solution = solve_bd(initial_state,0)
     optimal = 'yes'

solution_line = optimal
for each in solution:
        solution_line = solution_line + " " + str(each)
print solution_line
