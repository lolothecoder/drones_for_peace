from dijkstra import Graph, DijkstraSPF
from matplotlib import pyplot as plt
import numpy

nodes_points = []
length = 0.1
for i in range(2):
    for y in range(4):
        nodes_points.append((round(length*i, 1), round(length*y, 1)))
print(str(nodes_points))

graph = Graph()
for i in range(len(nodes_points) - 1):
    graph.add_egdge(nodes_points[i], nodes_points[i+1], 1)
    graph.add_egdge(nodes_points[i], nodes_points[i+1], 1)

graph.add_edge(0, 1, 1)
graph.add_edge(1, 2, 1)
graph.add_edge(2, 3, 1)
graph.add_edge(0, 4, 1)
graph.add_edge(1, 5, 1)
graph.add_edge(2, 6, 1)
graph.add_edge(3, 7, 1)
graph.add_edge(4, 5, 1)
graph.add_edge(5, 6, 1)
graph.add_edge(6, 7, 1)

dijkstra = DijkstraSPF(graph, 2)

print("%-5s %-5s" % ("label", "distance"))
for u in nodes:
    print("%-5s %8d" % (u, dijkstra.get_distance(u)))
#graph.remove_node(5)

#print(graph.get_data())
#nodes = list(graph.keys())

#connections = []
#for i in range(len(nodes)):
#    linked_nodes = list(graph[nodes[i]].keys())
#    for y in range(len(linked_nodes)):
#        connections.append([nodes[i], linked_nodes[y]])

#print("edges : " + str(connections))
#path = find_path(graph, 5, 0)
#print(path)
#print(dir(Graph))


points = numpy.array([[1,2],[4,5],[2,7],[3,9],[9,2]])
edges = numpy.array([[0,1],[3,4],[3,2],[2,4]])
x = points[:,0].flatten()
y = points[:,1].flatten()
plt.plot(x[edges.T], y[edges.T], 'y-') # Edges
plt.plot(x, y, 'ro') # Points
plt.show()

