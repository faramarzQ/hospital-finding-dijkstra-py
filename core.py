import sys

class Node:
    def __init__(self):
        self.shortest_distance = float('inf')

    def setShortestDistance(self, distance, edge):
        if(distance < self.shortest_distance):
            self.shortest_distance = distance
            self.edge = edge
    
class Edge:
    def __init__(self, from_, to_, distance):
        self.from_ = from_
        self.to_ = to_
        self.distance = distance

def findCurrentNodeNeighbors(edges, node_index, visited):
    neighbors = []
    for edge in edges:
        if(edge.from_ == node_index and edge.to_ not in visited):
            neighbors.append(edge.to_)
    return neighbors

def hasCommon(list1, list2): 
    result = False
    if(not list1 or not list2):
        return result

    for x in list1: 
        for y in list2: 
            if x == y: 
                result = True
                return result
    return result 

def findNearestOncommingNode(oncomming_nodes, nodes, hospital_nodes):
    nearest_distance = float("inf")
    nearest_node = None
    for index in oncomming_nodes:
        if(nodes[index].shortest_distance < nearest_distance):
            nearest_distance = nodes[index].shortest_distance
            nearest_node = index
    return nearest_node

def dijkstra():
    input_ = open("input.txt", "r").read().split('\n')
    hospitals_count = input_[0]
    hospital_nodes_str = input_[1].split(' ')
    driver_node = int(input_[2])
    nodes_count = int(input_[3])
    edges_count = input_[4]

    visited = []
    nodes = {}
    edges = []
    hospital_nodes = []
    oncomming_nodes = []
    current_node = driver_node

    # create object for every node
    for i in range(0, nodes_count):
        nodes[i] = Node()
    
    # create object for every edge
    for i in range(5, len(input_)):
        temp = input_[i].split(' ')
        obj = Edge(int(temp[0]), int(temp[1]), int(temp[2]))
        edges.append(obj)

    # remove probable enterd whitespaces in input files
    for node in hospital_nodes_str:
        if(node != ''):
            hospital_nodes.append(int(node))

    # in case the targe nodes are not in graph, kill the program
    list = []
    for edge in edges:
        list.append(edge.to_)
    if(not hasCommon(list, hospital_nodes)):
        sys.exit()

    nodes[current_node].setShortestDistance(0, None)
    while(not hasCommon(hospital_nodes, visited)): # untill it reaches the first target
        neighbors = findCurrentNodeNeighbors(edges, current_node, visited)

        # assign shortest distance to every neighbour.
        # there might be two edges between two nodes, sign the shortest one to next one.
        for neighbor_index in neighbors:
            for edge in edges:
                if(edge.from_ == current_node and edge.to_ == neighbor_index):
                    distance_to_neighbor = nodes[current_node].shortest_distance + edge.distance
                    nodes[neighbor_index].setShortestDistance(distance_to_neighbor, edge)

        oncomming_nodes += neighbors
        visited.append(current_node)
        oncomming_nodes = [x for x in oncomming_nodes if x != current_node]
        current_node = findNearestOncommingNode(oncomming_nodes, nodes, hospital_nodes)

    # calculate the path to target
    path = []
    previous = visited[-1]
    x = False
    while(x == False):
        for node in visited:
            if(previous == node):
                path.append(previous)
                if(nodes[node].edge):
                    previous = nodes[node].edge.from_
                    break
                else:
                    x = True

    print(path.reverse)

#run program
dijkstra()