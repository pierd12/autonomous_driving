#!/usr/bin/env python
# license removed for brevity

import rospy
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



def init_nodes():
     nodes = [[3.4,-0.39],#start0
     [3.4,-0.39],#A 1
     [3.4,0.08],#B 2
     [3.4,3.44],#C 3
     [3.4,3.9],#D 4
     [2.9,4.13],#E 5
     [0.869,4.237],#F 6
     [0.248,4.241],#G  7
     [-2.596,4.268],#H 8
     [-3.38,3.9],#I 9
     [-3.485,3.487],#J 10
     [-3.37,2.93],#K=red 11
     [0,0],#L 12
     [0,0],#M 13
     [0,0],#N 14
     [0,0],#O 15
     [0,0],#P
     [0,0],#R
     [0,0],#S
     [0,0],#W
     [0,0],#X
     [2.52,0.15],#B1
     [0,0],#B2
     [0.28,0.26],#B3
     [0.24,2.64],#B4=Purple
     [0,0],#B4
     [0,0],#B5
     [0,0],#B6
     [0,0],#B7
     [0,0],#B8
     [0,0],#B9
     [0,0]#B10
     ]
     return nodes
def init_graph():
    graph =  {"0": {"1","4"},#Start
    "1": {"21","4"},#A
    "2": {"4","21"},#B
    "3": {"4"},#C
    "4": {"7","9"},#D
    "5": {},#E
    "6": {},#F
    "7": {"9"},#G
    "8": {},#H
    "9": {"11"},#I
    "10": {}, #J
    "11": {"13","15"}, #RED
    "12": {},#L
    "13":{},#M
    "14":{},#N
    "15":{},#O
    "16":{},#P
    "17":{},#R
    "18":{},#S
    "19":{},#W
    "20":{},#X
    "21":{"23"},#B1
    "22":{},#B2
    "23":{"24"},#B3
    "24":{},#B4=PURPLE
    "25":{},#B4
    "26":{},#B5
    "27":{},#B6
    "28":{},#B7
    "29":{},#B8
    "30":{},#B9
    "31":{}#B10 
    }
    return graph
        
def find_path(graph, start, end, path):
    #blue x= 1.893092 y= -3.778001 z=-0.001009
    #green x= -0.359663 y= -0.498843
    #pink x=0.240103 y=2.573439
    #red x=-3.434260 y=2.952799
    path = path + [start]
    if start == end:
       return path
    if not graph.has_key(start):
       return None
    for node in graph[start]:
        if node not in path:
           newpath = find_path(graph, node, end, path)
           if newpath: 
	      return newpath
    return None

def movebase_client(distx,disty):
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = distx
    goal.target_pose.pose.position.y = disty
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    rospy.init_node('movebase_client_py')  
	
	#movebase_client(1)
    nodes = init_nodes()
    graph = init_graph()
    path = find_path(graph,"0","24",[])
    print (path)
    for x in path:
        x = int(x)
        print("driving to the goal through {}".format(x))
        node = nodes[x]
	distx = node[0]
	disty = node[1]
        result = movebase_client(distx,disty)
        if result:
            rospy.loginfo("Got through {}!".format(x))
    rospy.loginfo("Goal execution done!")










