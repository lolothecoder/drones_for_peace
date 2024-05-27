from dijkstar import Graph, find_path
from matplotlib import pyplot as plt
import numpy

length = 0.1
rows = 8
columns = 4
start = 2
goal = 28

def update_connections(graph):
    nodes = list(graph.keys())
    connections = []
    for i in range(len(nodes)):
        linked_nodes = list(graph[nodes[i]].keys())
        for y in range(len(linked_nodes)):
            connections.append([nodes[i], linked_nodes[y]])
    return connections

def remove(graph, node):
    graph.remove_node(node)
    connections = update_connections(graph)
    return connections

def create_graph(rows, columns, length):
    nodes_points = []
    for i in range(rows):
        for y in range(columns):
            nodes_points.append((round(length*y, 1), round(length*i, 1)))

    graph = Graph()
    for i in range(len(nodes_points)):
        if ((i < len(nodes_points) - 1) and (nodes_points[i][1] == nodes_points[i+1][1])):
            graph.add_edge(i, i+1, 0.1)
            graph.add_edge(i+1, i, 0.1)
        if (nodes_points[i][1] > 0):
            graph.add_edge(i, i-4, 0.1)
            graph.add_edge(i-4, i, 0.1)

    connections = update_connections(graph)
    return graph, nodes_points, connections 

def get_best_path(graph, node_points, start, goal):
    best_path_edges = []
    best_path_nodes = []

    path = None
    while path is None:
        try:
            path = find_path(graph, start, goal)
        except:
            goal += 1
            print("Changed goal")

    for i in range(len(path.nodes)):
        if(i < len(path.nodes) - 1):
            best_path_edges.append([i,i+1])
        best_path_nodes.append(node_points[path.nodes[i]])
    return best_path_edges, best_path_nodes

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

    plt.plot(x[edges.T], y[edges.T], 'y-') # Edges
    plt.plot(x, y, 'ro') # Points

    plt.plot(x_best[best_path_edges.T], y_best[best_path_edges.T], 'b-') # Edges
    plt.plot(x_best, y_best, 'bo') # Points
    plt.axis('equal')
    plt.show()

graph, node_points, connections = create_graph(rows, columns, length)
connections = remove(graph, 8)
connections = remove(graph, 9)
connections = remove(graph, 13)
connections = remove(graph, 20)
best_path_edges, best_path_nodes = get_best_path(graph, node_points, start, goal)
visualisations(node_points, connections, best_path_nodes, best_path_edges)


