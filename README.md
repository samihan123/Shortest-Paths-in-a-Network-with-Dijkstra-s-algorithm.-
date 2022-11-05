# Shortest-Paths-in-a-Network-Using-Dijkstra-s-algorithm.


**ITCS-6114/8114: Algorithms and Data Structures**

**Brief Description**
In this project, I am building a data communication network. In data communication, routers are connected with the transmitting line. 
Transmit line will have the transmitting time required to transmit the data. We need to compute the best path using the criterion of minimizing 
the total time taken for data to reach the destination. We can have a scenario where transmit link or router can be down. If it is down we won't be able to traverse through that path. 

In this program, the whole network is implemented in the graph. Vertex represents the router and edges represent the transmitting link. 
Edges have the source vertex and destination vertex. To find the shortest path between the vertices, we have implemented Dijkstra's algorithm. 
To find a vertex that is reachable we have implemented a Depth-first search on each vertex.

The program would implement the following things
1. Building initial graph.
    Initially, with the command *graph <filename>*, we would be able to plot the graph. This command would take the parameter as the file name. 
    In the file, each line represents two directed edges specified by two vertices followed by transmitting time. This would be read by the program and a graph would be constructed 
2. Updates to the graph.
    a. *added tailvertex headvertex transmit time*, this command adds an edge to the graph. if edges are already present, the transmitting time would be changed. 
    b. *deleteedge tailvertex headvertex*, this command deletes an edge from the graph. Only the edge would be deleted, vertices won't be deleted.
    c. *edgedown tailvertex headvertex*, this command makes that edge down. If the edge is down, we won't be able to traverse the graph.
    d. *edgeup tailvertex headvertex*, this command makes that edge up
    e. *vertexdown vertex*, this command makes the vertex down. When the vertex is down we won't be able to traverse through that vertex.
    f. *vertexdown vertex*, This command makes the vertex up.

3. Find the Shortest Path.
    To find the shortest path, we are implementing Dijkstra's algorithm. In Dijkstra's algorithm, we are maintaining a heap queue to store all the vertices according to their weight. 
command *path from_vertex to_vertex* will give us the shortest path and distance between the two vertexes. We have implemented the following algorithm for Dijkstra.
        a. resetting all vertices values. (setting distance as inf and previous  as None)
        b. Set the distance of the source vertex to 0.
        c. add all vertex to heap queue.
        d. while the queue is empty.
            a.U = heap pop() // this would pop the minimum element.
            b. for all V the adjacent vertices of U
                a.if distance (V) > distance(U) + weight (U,V)
                    assign distance(U) + weight (U,V) to vertex V
                    set previous of V to U.
                    as distance changes, we need to implement 	heap_decrease_priority.
    
    When the value inside the heap has changed we need to implement a heap decrease priority algorithm. In heap decrease priority, 
    elements are swapped with parents if the value of parent is greater than its children from the index of value which has been changed.

4. Printing the graph
    command *print* will print the content of the graph. Vertices and their edges would be printed in alphabetical order.
5. Finding reachable sets of vertices.
    command *reachable* will print all the reachable vertices from each vertex. We have implemented a Depth-first search algorithm to find all reachable vertex.
    DFS(vertex V)
    a. for U in adj neighbors of V:
    b. If U is not visited, not down and edge(U,V) is not down
        Then vertex U is reachable from V
        DFS(U)


 
**Break down**
  
  
Command to run the program.
python graph.py
When the program starts execution, it would take input from the command line.
The program consists of 1 file named graph.py. 

Python files consist of the following 3 classes.
1. **Vertex Class:**
    	Vertex object is created using this class. This consists of the blueprint of the vertex.
    	Vertex class consists of the following 7 variables.:
    		- name - This is used to store the Name of the vertex.
        	- adj_vertex - This is used to store a list of adjacent vertex objects.
        	- active - This is used to store the UP/DOWN status of the vertex. 
        	- edges - This is used to store the list of edges objects, of which vertex is the source.
        	- prev - This is used to store an object of the previous/parent node while running Dijkstra's algorithm.
        	- distance - This is used to store distance from source to this vertex while running the Dijkstra algorithm.
        	- visited - This is used to store the status of visited while running Dijkstra's algorithm.
2. **Edge Class:**
	Edge object is created using this class. This consists of the blueprint of edge.
	Edge class consists of the following variables.
       - src - This is used to store source vertex objects.
       - dest – This is used to store destination vertex objects.
       - transmit_time – This is used to store transmit time required to travel from source to destination.
       - active - This is used to store the UP/DOWN status of the vertex. 


3. **Graph Class:**
	Graph class is used to create a graph, evaluate Dijkstra, and reachable algorithm.
	Graph class consists of the following variables.
	    - vertexMap – This is a dictionary that stores key as vertex name and value as an object of the vertex.
        - edgeMap - This is a dictionary that stores key as vertex name of source and destination in tuple format and value as an object of the Edge.

Graph class consists of the following functions.
1.	addEdge(headvertex,tailvertex,trasmit_time)
	- This function is used to add an edge to the graph. If the edge is already present it would change its transmit time.
	Time Complexity of add edge is O(1)
2.	print_graph():
	- This function is used to print the content of the graph, vertices and outward edges are printed in alphabetical order.
3.	deleteedge(headvertex,tailvertex):
	- This function is used to remove the specified directed edge from the graph. Do not delete vertex. If there are no edges present, do nothing. The time complexity of this function would be O(1).
4.	edgedown(headvertex,tailvertex):
	- This function is used to mark the directed edge as down and it would be unavailable to use. The time complexity of this function would be O(E).
5.	edgeup(headvertex,tailvertex):
	- This function is used to mark the directed edge as up and it would be available to use. The time complexity of this function would be O(E).
6.	vertexdown(vertex):
	- This function is used to make the vertex down. When the vertex is down, none of the edges can pass through it. The time complexity of this function would be O(V).
7.	vertexup(vertex):
	- This function is used to make vertex up. The time complexity of this function would be O(V).
8.	shortestPath(self,source,destination):
	- This function is used to call the dijikstra function and then would print the shortest path.
9.	dijikstra(self,source):
	- This function would implement the dijikstra algorithm. This would construct the Dijkstra graph to find the shortest path. This algorithm would run in O((V+E)Log V)
10.	heap_decrease_key(self,v,alt,bin_heap_q):
	- This function would implement heap decrease priority. This function would be called for Dijkstra's algorithm when we decrease the distance value of the vertex.This algorithm would run in O(log V)
11.	reachable():
	- This function is used to implement the Depth First Search algorithm to find all reachable vertices. This algorithm would run in O(V*(V+E))



**A summary of things would fails in my program**
Dijkstra algorithm work on non-negative edges. Dijkstra algorithm won't work on negative edges. Dijkstra algorithm always chooses the shortest path. When computing a Dijkstra between two vertices, each additional traversal along the cycle lowers the overall cost incurred and an arbitrarily small distance can be reached after looping around the cycle multiple times. In case of a negative edge, we can get stuck in a loop.

**Data structure design.**
There are mainly 2 data structures used in the program
1. Graph
2. Heap queue

**Graph:**
A Graph is a nonlinear data structure consisting of vertices and edges. Vertices are connected by the edge. Edge will have its weight. In our project, we have implemented a data communication network in graph format. The graph can be directed or undirected.

We have implemented the following things on the graph.
1. Dijkstra algorithm to find the shortest path.
2. Depth-first search to get all the reachable vertexes.

**Heap Queue:**
Heap queue can be classified into 2:
1. Min heap Queue
    In this Heap queue which parent node is equal to or smaller than its children.
2. Max heap Queue
    In this Heap queue which parent node is equal to or greater than its children.
We have implemented a heap queue in Dijkstra's algorithm to store the distance of vertex.
Heap mainly consists of the following functions.
- Heapify - use to convert the list into a heap
- heap pop - use to convert the smallest element from the list. After the element is pop, list would heapified again,
- heap decrease priority - When we decrease the value in the list, we need to run heap decrease priority. In this algorithm, elements are swapped with parents if the value of the parent is greater than its children from the index of value which has been changed.
