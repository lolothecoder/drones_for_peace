from dijkstar import Graph, find_path
from matplotlib import pyplot as plt
import numpy
import threading
import time

'''
# Task to be executed at regular intervals.
def timer():
    while True:
        print("Hello world!")
        time.sleep(3)   # 3 seconds.
'''
# Start execution in the background.
#t = threading.Thread(target=timer)
#t.start()

'''
length = 0.15
rows = 8
columns = 7
start = 2
goal = 28
'''


def print_msg():
    print("Mmmmmm Selma")

def update_connections(graph):
    nodes = list(graph.keys())
    connections = []
    for i in range(len(nodes)):
        linked_nodes = list(graph[nodes[i]].keys())
        for y in range(len(linked_nodes)):
            connections.append([nodes[i], linked_nodes[y]])
    return connections

def remove(graph, node):
    #print(graph)
    #print(type(node))
    try:
        graph.remove_node(node)
        connections = update_connections(graph)
        return connections
    except:
        return None

def create_graph(rows, columns, length):
    nodes_points = []
    for i in range(rows):
        for y in range(columns):
            nodes_points.append((round(length*y, 2), round(length*i, 2)))

    graph = Graph()
    for i in range(len(nodes_points)):
        if ((i < len(nodes_points) - 1) and (nodes_points[i][1] == nodes_points[i+1][1])):
            graph.add_edge(i, i+1, length)
            graph.add_edge(i+1, i, length)
        if (nodes_points[i][1] > 0):
            graph.add_edge(i, i-columns, length)
            graph.add_edge(i-columns, i, length)

    connections = update_connections(graph)
    return graph, nodes_points, connections 

def get_best_path(graph, node_points, start, goal):
    best_path_edges = []
    best_path_nodes = []

    path = None
    while path is None:
        try:
            print("trying")
            path = find_path(graph, start, goal)
        except:
            goal += 1
            if goal > len(graph): goal = 0
            print("Changed goal : " + str(goal))
        #print(path)

    for i in range(len(path.nodes)):
        if(i < len(path.nodes) - 1):
            best_path_edges.append([i,i+1])
        best_path_nodes.append(node_points[path.nodes[i]])
    node_ids = path.nodes
    return best_path_edges, best_path_nodes, node_ids

def visualisations(node_points, connections, best_path_nodes, best_path_edges):
    points = numpy.array(node_points)
    edges = numpy.array(connections)
    best_path_nodes = numpy.array(best_path_nodes)
    best_path_edges = numpy.array(best_path_edges)

    #print("edges : " + str(edges))
    x = points[:,0].flatten()
    #print("x : " + str(x))
    y = points[:,1].flatten()

    x_best = best_path_nodes[:,0].flatten()
    y_best = best_path_nodes[:,1].flatten()

    plt.ion()
    fig = plt.figure()
    plt.plot(x[edges.T], y[edges.T], 'y-') # Edges
    plt.plot(x, y, 'ro') # Points

    plt.plot(x_best[best_path_edges.T], y_best[best_path_edges.T], 'b-') # Edges
    plt.plot(x_best, y_best, 'bo') # Points
    plt.axis('equal')
    #fig = plt.gcf() 
    return fig

def generate_dijkstra(rows, columns, length, start, goal):
    graph, node_points, connections = create_graph(rows, columns, length)
    best_path_edges, best_path_nodes, node_ids = get_best_path(graph, node_points, start, goal)
    fig = visualisations(node_points, connections, best_path_nodes, best_path_edges)
    return graph, node_points, connections, best_path_edges, best_path_nodes, fig, node_ids

def conversion(node_points, height):
    sequence = []
    for i in range(len(node_points)):
        sequence.append((node_points[i][1], -node_points[i][0], height))
    return sequence

def draw_map(fig):
    fig.canvas.draw() 
    fig.canvas.flush_events() 
#graph, node_points, connections, best_path_edges, best_path_nodes = generate_dijkstra(rows, columns, length, start, goal)
#print(best_path_nodes)
#print(best_path_nodes[0][0], best_path_nodes[0][1])
#converted_nodes = conversion(best_path_nodes, 0.4)
#print(converted_nodes)
#node_points = numpy.array([(1, 1), (2, 2)])
#converted = conversion(node_points)
#print(converted)

#plt.close(fig)

'''
graph, node_points, connections = create_graph(rows, columns, length)
#print(graph)
#print(node_points)
#print(connections)
best_path_edges, best_path_nodes, node_ids = get_best_path(graph, node_points, start, goal)
print(best_path_edges)
print(best_path_nodes)
print(node_ids)
fig = visualisations(node_points, connections, best_path_nodes, best_path_edges)
draw_map(fig)
time.sleep(30)

#plt.close(fig)
#fig.close()
#plt.show()
#plt.close(fig)
#connections = remove(graph, 8)
#connections = remove(graph, 9)
#connections = remove(graph, 13)
#connections = remove(graph, 20)
print("wohoo")
#best_path_edges, best_path_nodes = get_best_path(graph, node_points, start, goal)
print("good")
#fig = visualisations(node_points, connections, best_path_nodes, best_path_edges)
#draw_map(fig)
#fig.show()
print("ok")
'''