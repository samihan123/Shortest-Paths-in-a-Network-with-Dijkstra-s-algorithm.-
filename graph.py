# -*- coding: utf-8 -*-
"""
Created on Tue Nov 16 16:34:17 2021

@author: Samihan Jawalkar
@student ID: 801254625

"""
import numpy as np
import sys
import heapq
import os

class Vertex:
    def __init__(self,name, distance=np.inf,active=True):
        '''
        Vertex Class

        Parameters
        ----------
        name : string
            name of vertex.
        distance : number, optional
            to calculate distance from src vertex. The default is np.inf.
        active : bool, optional
            status for Up/Down. The default is True.

        Returns
        -------
        None.

        '''
        self.name = name
        self.adj_vertex = []
        self.active  = active
        self.edges = []
        self.prev = None
        self.distance = np.inf
        self.visited = False
    def __lt__(self,other):
        return self.distance<other.distance
    def reset_paths(self):
        '''
        reseting distance and previous

        Returns
        -------
        None.

        '''
        self.distance = np.inf
        self.prev = None
    
    def reset_visits(self):
        '''
        Reseting visits

        Returns
        -------
        None.

        '''
        self.visited = False

class Edge:
    def __init__(self,src,dest,transmit_time,active=True):
        '''
        Edge class

        Parameters
        ----------
        src : object
            object of src variable.
        dest : object
            object of destination variable.
        transmit_time : number
            edge weight.
        active : bool, optional
            status for Up/Down. The default is True.

        Returns
        -------
        None.

        '''
        self.src = src
        self.dest = dest
        self.transmit_time = float(transmit_time)
        self.active = active

class Graph:
    def __init__(self):
        '''
        Graph class to create graph

        Returns
        -------
        None.

        '''
        self.vertexMap =  dict()
        self.edgeMap = dict()
    
    ##########################################################################
    #add edge to the graph, if edge is already present change the transmit_time
    #time complexity = O(V+E)
    ##########################################################################
    def addEdge(self,headvertex,tailvertex,trasmit_time):
        '''
        add edge to graph

        Parameters
        ----------
        headvertex : object
            object of head vertex.
        tailvertex : object
            object of tail vertex.
        trasmit_time : TYPE
            edge weight.

        Returns
        -------
        None.

        '''
        #create vertices
        v = self.getVertex(headvertex)        
        w = self.getVertex(tailvertex)
        #assigning adj vertex
        if w not in v.adj_vertex:
            v.adj_vertex.append(w)
        #create edges
        e = self.getEdge(v,w,trasmit_time)
        
        #assigning edge to vertex 
        if e not in v.edges:
            v.edges.append(e)
    
    ##########################################################################
    # If vertexName is not present, add it to vertexMap.
    # In either case, return the Vertex.
    #time complexity = O(V)
    ##########################################################################
    def getVertex(self, vertexName):
        '''
        

        Parameters
        ----------
        vertexName : str
            name of vertex.

        Returns
        -------
        v : obj
            vertex obj.

        '''
        if vertexName not in self.vertexMap:
            v = Vertex(vertexName)
            self.vertexMap[vertexName] = v
        v = self.vertexMap[vertexName]
        return  v
    
    ##########################################################################
    # If Edges is not present, add it to EdgeMap.
    #if present change its trasmi
    # In either case, return the edge.
    #Time Complexity = O(E)
    #########################################################################
    def getEdge(self,src,dest,transmit_time):
        '''
        

        Parameters
        ----------
        src : object
            object of source.
        dest : object
            object if destination.
        transmit_time : number
            transmit time weight of graph.

        Returns
        -------
        e : obj
            edge object.

        '''
        vertex_tup = (src.name,dest.name)
        if vertex_tup not in self.edgeMap:
            e = Edge(src,dest,transmit_time)
            self.edgeMap[vertex_tup] = e
        else:
            e = self.edgeMap[vertex_tup]
            e.transmit_time = float(transmit_time)
        return e
    
    
    #########################################################################
    #print content of graph, vetices and outward edges are printed in
    #alphabetical order
    #Time complexity = O(V+E)
    #########################################################################
    def print_graph(self):
        '''
        Prints graph in given format

        Returns
        -------
        None.

        '''
        
        sorted_vertex = {k:v for k,v in sorted(self.vertexMap.items())}
        for v in sorted_vertex:
            src_vertex_obj = sorted_vertex[v]
            print("{sv} {st}".format(sv=src_vertex_obj.name
                                     ,st="" if src_vertex_obj.active else "down"))
            #print all adj vertex
            all_edges = src_vertex_obj.edges
            all_edges.sort(key=lambda x: x.dest.name)
            #all_edges = all_edges.sorted(key)
            for e in all_edges:
                print("\t{dn} {tt} {st}".format(dn=e.dest.name
                                                ,tt=e.transmit_time
                                                ,st="" if e.active else "down"))
                
    ##########################################################################
    #check if edge is present, if present delete edge
    #vertices wont be removed, if vertices does not exist do nothing.
    #Time complexity = O(E)
    ##########################################################################
    def deleteedge(self,headvertex,tailvertex):
        '''
        delete edge if present

        Parameters
        ----------
        headvertex : TYPE
            DESCRIPTION.
        tailvertex : TYPE
            DESCRIPTION.

        Returns
        -------
        None.

        '''
        vertex_tup = (headvertex,tailvertex)
        if vertex_tup in self.edgeMap:
            edge_obj = self.edgeMap[vertex_tup]
            vertex_obj_src = self.vertexMap[headvertex]
            vertex_obj_dest = self.vertexMap[tailvertex]
            #deleting from vertex obj
            vertex_obj_src.edges.remove(edge_obj)
            vertex_obj_src.adj_vertex.remove(vertex_obj_dest)
            #deleting for edge map
            del self.edgeMap[vertex_tup]
        else:
            pass
            #print("Edge not found in graph")
    
    ##########################################################################
    #check if edge is present, if present make edge down/Unavailble to use
    #Time complexity = O(E)
    #########################################################################
    def edgedown(self,headvertex,tailvertex):
        '''
        

        Parameters
        ----------
        headvertex : str
            name of head vertex.
        tailvertex : str
            name of tail vertex.

        Returns
        -------
        None.

        '''
        if (headvertex,tailvertex) in self.edgeMap:
            edge_obj = self.edgeMap[(headvertex,tailvertex)]
            edge_obj.active = False
        else:
            print("Edge not found in graph")
    ##########################################################################        
    #check if edge is present, if present make edge up/make it available to use
    #Time complexity = O(E)
    ##########################################################################
    def edgeup(self,headvertex,tailvertex):
        '''
        

        Parameters
        ----------
        headvertex : str
            name of head vertex.
        tailvertex : str
            name of tail vertex.

        Returns
        -------
        None.

        '''
        if (headvertex,tailvertex) in self.edgeMap:
            edge_obj = self.edgeMap[(headvertex,tailvertex)]
            edge_obj.active = True
        else:
            print("Edge not found in graph")
            
    ##########################################################################        
    #check if vertex is present, if present make vertex down. 
    #this vertex would not be available to use
    #Time complexity = O(V)
    ##########################################################################
    def vertexdown(self,vertex):
        '''
        

        Parameters
        ----------
        vertex : str
            name of vertex.

        Returns
        -------
        None.

        '''
        if vertex in self.vertexMap:
            vertex_obj = self.vertexMap[vertex]
            vertex_obj.active = False
        else:
            print("vertex not found in graph")
            
    ##########################################################################        
    #check if vertex is present, if present make vertex up
    #Time Complexity = O(V)
    ##########################################################################
    def vertexup(self,vertex):
        '''
        

        Parameters
        ----------
        vertex : str
            name of vertex.

        Returns
        -------
        None.

        '''
        if vertex in self.vertexMap:
            vertex_obj = self.vertexMap[vertex]
            vertex_obj.active = True
        else:
            print("vertex not found in graph")
            
    ##########################################################################
    #clear all distances
    #time complexity = O(V)
    #########################################################################
    def clearAllDistance(self):
        for v in self.vertexMap.values():
            v.reset_paths() 
    
    ##########################################################################
    #Dijikstra algortihm, 
    #Find the shortest path to all vertex from the give src
    #Time complexity = O(V+E)logV)
    ##########################################################################
    def dijikstra(self,source):
        '''
        

        Parameters
        ----------
        source : str
            name of sorce.

        Returns
        -------
        None.

        '''
        #resetting distance to inf and prev vertex to none        
        self.clearAllDistance()
        
        #setting src vertex to zero
        src_vertex_obj = self.vertexMap[source]
        src_vertex_obj.distance = 0
        
        #get the list of vertex object
        bin_heap_q = list(self.vertexMap.values())
        #heapify the list, heap queue is generated
        heapq.heapify(bin_heap_q)
        while len(bin_heap_q) != 0:
            u = heapq.heappop(bin_heap_q)
            
            for v in u.adj_vertex:

                #get the alternate distance
                if(v.active == False):
                    #dist_u_v = np.inf
                    continue
               
                #get edge object of v,u
                edge_obj = self.edgeMap[(u.name,v.name)]
                
                if(edge_obj.active == False):
                    continue
            
                alt = round(u.distance + edge_obj.transmit_time,3)
                #set the smallest distance
                if alt < v.distance:
                    v.distance = alt
                    v.prev = u
                    #heapq.heapify(bin_heap_q)
                    self.heap_decrease_key(v,alt,bin_heap_q)
    
    ##########################################################################
    #change value of given element to alt value, and run heap decrease key algoritm
    #Time Complexity = O(log V)
    ##########################################################################               
    def heap_decrease_key(self,v,alt,bin_heap_q):
        '''
        

        Parameters
        ----------
        v : object
            vertex object.
        alt : number
            alternte weight.
        bin_heap_q : list
            heap queue.

        Returns
        -------
        None.

        '''

        idx =  bin_heap_q.index(v)
        bin_heap_q[idx].distance=alt
        #checking with parent
        while idx > 0 and bin_heap_q[((idx+1)//2)-1].distance > bin_heap_q[idx].distance:
            bin_heap_q[((idx+1)//2)-1],bin_heap_q[idx] = bin_heap_q[idx],bin_heap_q[((idx+1)//2)-1]
            idx = ((idx+1)//2)-1
    
    ##########################################################################
    #  print total distance. It calls recursive routine to print 
    # shortest path to destNode after a shortest path algorithm has run.
    #Time Complexity = O(V+E)
    #############################################################################
    def printPath(self, destName):
        '''
        

        Parameters
        ----------
        destName : str
            destination name.

        Returns
        -------
        None.

        '''
        if(destName not in self.vertexMap):
            print("Destination not found")
        else:
            w = self.vertexMap[destName]
            if w is None:
                print("Destination vertex not found")
            elif np.isinf(w.distance):
                print("{dn} is unreachable".format(dn=destName))
            else:
                #print("Distance is {dt} ".format(dt=w.distance))
                #print("(Distance is: " + str(w.distance) + ") ", end ="")
                self.printPath_(w)
                print(" {dn}".format(dn=w.distance))
                

    
    ###########################################################################                
    # Recursive routine to print shortest path to dest
    # after running shortest path algorithm. The path
    # is known to exist.
    ###########################################################################
    def printPath_(self, dest):
        if dest.prev is not None:
            self.printPath_(dest.prev)
            print(" ", end ="")
        print(dest.name, end ="")
       
    ##########################################################################    
    #find the shortest path usingn dijikstra, and then print the path
    #Time Complexity = O((V+E)logv)
    ##########################################################################
    def shortestPath(self,source,destination):
        '''
        

        Parameters
        ----------
        source : str
            name of source vertex.
        destination : str
            name of destination vertex.

        Returns
        -------
        None.

        '''
        if(source not in self.vertexMap):
            print("Source not found")
        elif(destination not in self.vertexMap):
            print("Destination not found")
        else:
            #construct dijikstra
            self.dijikstra(source)
            
            #get the path
            self.printPath(destination)

    
    ###########################################################################
    #clear all visits
    #time complexity = O(V)
    ###########################################################################
    def clearAllVisits(self):
        for v in self.vertexMap.values():
            v.reset_visits()
    
    ###########################################################################
    #find all nodes which are reachable
    #time complexity O(V*(V+E))
    ###########################################################################
    def reachable(self):
        reachable = {}
        sorted_vertex = {k:v for k,v in sorted(self.vertexMap.items())}
        for v in sorted_vertex:
            self.clearAllVisits()
            
            
            if self.vertexMap[v].active == True:
                reachable[v]=[]
                self.dfsUtil(self.vertexMap[v],v,reachable)
            #   print("\n")
           
        #print
        for v in reachable:
            print(v)
            reachable[v].sort()
            for u in reachable[v]:
                print("\t",u)
    
    ##########################################################################
    #dfs algortihm for finding reachable vertices
    #time complexity = O(V+E)
    ##########################################################################
    def dfsUtil(self,vertex,v_name,reachable):
         vertex.visited = True
         for neighbour in vertex.adj_vertex:
             edge_obj = self.edgeMap[(vertex.name,neighbour.name)]
             if (neighbour.visited == False and edge_obj.active == True and neighbour.active==True):
                 #print("\t",neighbour.name)
                 reachable[v_name].append(neighbour.name)
                 self.dfsUtil(neighbour,v_name,reachable)
         
##############################################################################
#main function
##############################################################################       
def main():
    try:
        g=None
        while(True):
            try:
                cmd = input()
                cmd_list = cmd.strip().split(" ")
                
                #1.Building the initial graph.
                #graph network.txt
                if(cmd_list[0] == "graph"):
                    if(len(cmd_list)==2):
                        #creating graph
                        g = Graph()
                        fin = cmd_list[1]
                        if(os.path.exists(fin)):
                            with open(fin) as f:
                                lines = f.readlines()
                                for line in lines:
                                    line = line.strip().split(" ")
                                    if (len(line) != 3):
                                        print("Skipping ill-formatted line " + line)
                                        continue
                                    
                                    vertex_1 = line[0]
                                    vertex_2 = line[1]
                                    transmit_time = line[2]
                                    g.addEdge(vertex_1, vertex_2, transmit_time)
                                    g.addEdge(vertex_2, vertex_1, transmit_time)
                        else:
                            print("File '{fl}' not found".format(fl=fin))
                            continue
                        
                    else:
                        print("Expected 1 parameter , {n} found".format(n=len(cmd_list)-1))
                        continue
                
                #2.Updating the graph to reect changes.
                #2a edge down
                elif(cmd_list[0] == "edgedown"):
                    if(len(cmd_list)==3):
                        if(g!=None):
                            g.edgedown(cmd_list[1],cmd_list[2])
                        else:
                            print("No graph created yet")
                    else:
                        print("Expected 2 parameter , {n} found".format(n=len(cmd_list)-1))
                        continue
                #2b edge up
                elif(cmd_list[0] == "edgeup"):
                    if(len(cmd_list)==3):
                        if(g!=None):
                            g.edgeup(cmd_list[1],cmd_list[2])
                        else:
                            print("No graph created yet")
                    else:
                        print("Expected 2 parameter , {n} found".format(n=len(cmd_list)-1))
                        continue
                #2c vertex up
                elif(cmd_list[0] == "vertexup"):
                    if(len(cmd_list)==2):
                        if(g!=None):
                            g.vertexup(cmd_list[1])
                        else:
                            print("No graph created yet")
                    else:
                        print("Expected 1 parameter , {n} found".format(n=len(cmd_list)-1))
                        continue
                    
                #2d vertex down
                elif(cmd_list[0] == "vertexdown"):
                    if(len(cmd_list)==2):
                        if(g!=None):
                            g.vertexdown(cmd_list[1])
                        else:
                            print("No graph created yet")
                    else:
                        print("Expected 1 parameter , {n} found".format(n=len(cmd_list)-1))
                        continue
                #2e delete
                elif(cmd_list[0] == "deleteedge"):
                    if(len(cmd_list)==3):
                        if(g!=None):
                            g.deleteedge(cmd_list[1],cmd_list[2])
                        else:
                            print("No graph created yet")
                    else:
                        print("Expected 2 parameter , {n} found".format(n=len(cmd_list)-1))
                        continue
                #2f add edge
                elif(cmd_list[0]=='addedge'):
                    if(len(cmd_list)==4):
                        if(g==None):
                            g = Graph()
                        g.addEdge(cmd_list[1],cmd_list[2],cmd_list[3])
                    else:
                        print("Expected 3 parameter , {n} found".format(n=len(cmd_list)-1))
                #3.Finding the shortest path between any two vertices in the graph based on its current state.
                elif(cmd_list[0]=='print'):
                    if(len(cmd_list)==1):
                        if(g==None):
                            print("Graph not created yet")
                            continue
                        g.print_graph()
                    else:
                        print("Expected 0 parameter , {n} found".format(n=len(cmd_list)-1))
                #4.printing the graph.
                elif(cmd_list[0]=='path'):
                    if(len(cmd_list)==3):
                        if(g!=None):
                            g.shortestPath(cmd_list[1],cmd_list[2])
                        else:
                            print("No Graph created yet")
                    else:
                        print("Expected 2 parameter , {n} found".format(n=len(cmd_list)-1))
                
                #5.Finding reachable sets of vertices.
                elif(cmd_list[0]=='reachable'):
                    g.reachable()
                #6 exit
                elif(cmd_list[0]=='quit'):
                    print("Bye!!!")
                    break
                else:
                    print("Invalid command")
                    continue
            except:
                print("Error: ",sys.exc_info())
                continue
    except:
            print("Error while starting ",sys.exc_info())
        
    
    
main()
    
        
        