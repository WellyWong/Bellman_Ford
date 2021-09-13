"""
The Bellman–Ford algorithm is an algorithm that computes shortest paths from a single source vertex to all of the other
vertices in a weighted digraph. It is slower than Dijkstra's algorithm for the same problem, but more versatile,
as it is capable of handling graphs in which some of the edge weights are negative numbers.

The Bellman–Ford algorithm is an algorithm that computes shortest paths from a single source vertex to all of the other
vertices in a weighted digraph. It is slower than Dijkstra's algorithm for the same problem, but more versatile,
as it is capable of handling graphs in which some of the edge weights are negative numbers.
Negative edge weights are found in various applications of graphs, hence the usefulness of this algorithm. If a graph
contains a "negative cycle" (i.e. a cycle whose edges sum to a negative value) that is reachable from the source, then
there is no cheapest path: any path that has a point on the negative cycle can be made cheaper by one more walk
around the negative cycle. In such a case, the Bellman–Ford algorithm can detect and report the negative cycle.
Like Dijkstra's algorithm, Bellman–Ford proceeds by relaxation, in which approximations to the correct distance
are replaced by better ones until they eventually reach the solution. In both algorithms, the approximate distance to
each vertex is always an overestimate of the true distance, and is replaced by the minimum of its old value and
the length of a newly found path. However, Dijkstra's algorithm uses a priority queue to greedily select the closest
vertex that has not yet been processed, and performs this relaxation process on all of its outgoing edges;
by contrast, the Bellman–Ford algorithm simply relaxes all the edges, and does this |V|-1 times, where |V| is the number
of vertices in the graph. In each of these repetitions, the number of vertices with correctly calculated distances grows
from which it follows that eventually all vertices will have their correct distances. This method allows
the Bellman–Ford algorithm to be applied to a wider class of inputs than Dijkstra. The intermediate answers depend on
the order of edges relaxed, but the final answer remains the same.

Bellman–Ford runs in O(|V||E|) time, where |V| and |E| are the number of vertices and edges respectively.
"""
import math

def initialize(graph, source):
    d = {}   # shortest-path estimate
    p = {}   # parent/predecessor
    for node in graph:
        d[node] = float('Inf')  #float('inf') is used for setting a variable with an infinitely large value
                                # alternatively, we can use sys.maxsize
        p[node] = None
    d[source] = 0
    return d, p

def relax(u, v, graph, d, p):
    if d[v] > d[u] + graph[u][v]:
        d[v] = d[u] + graph[u][v]
        p[v] = u

def bellman_ford(graph, source):
    d, p = initialize(graph, source)
    for i in range(len(graph)-1):   #Run this until is converges
        for u in graph:
            for v in graph[u]:      #For each neighbour of u
                relax(u, v, graph, d, p)    #Lets relax it

    # Step 3: check for negative-weight cycles
    for u in graph:
        for v in graph[u]:
            #assert d[v] <= d[u] + graph[u][v]
            if d[v] > d[u] + graph[u][v]:
                print("Graph contains a negative-weight cycle, {} --> {}".format(u, v))
                d[v] = -math.inf
    return d, p


graph = {'a': {'b': -1, 'c':  4},
        'b': {'c':  3, 'd':  2, 'e':  2},
        'c': {},
        'd': {'b':  1, 'c':  5},
        'e': {'d': -3}
        }

d, p = bellman_ford(graph, 'a')
print(d)
print(p)
print(graph)
print(float('Inf'))

g = {'S': {'A': 10, 'E': 8}, 'A': {'C': 2}, 'B': {'A': 1}, 'E': {'D': 1}, 'D': {'C': -1, 'A': -4}, 'C': {'B': -2}}
d, p = bellman_ford(g, 'S')
print(d)
print(p)

"""
in order for d[v] to be updated at a particular 
iteration, there must be an edge (u,v) such that d[u]+w(u,v)<d[v]. That is, we must be able to improve our estimation of
the distance from s to v in order to update d[v]. In the first iteration, the value of d[u]=inf 
for every vertex u (except s). Therefore, if v is not a neighbor of s, then u is not s, and hence the value of 
d[u]+w(u,v) equals inf+w(u,v)=inf. This means we cannot improve our estimation of d[v]. This is why only the neighbors 
of s are updated in the first iteration even though the algorithm iterates over all the edges of the graph.
"""
"""
Single Source Shortest Path algorithm: DAG  --->>> O(V+E) time
https://www.youtube.com/watch?v=2E7MmKv0Y24   at 18:40
DAGs : Directed Acyclic Graphs --->>> can not have negative cycles
What causes problem is negative cycle, not negative edge

'For a general weighted graph, we can calculate single source shortest path in O(VE) time using Bellman–Ford Algorithm.
For a graph with no negative weights, we can do better and calculate single source shortest distances
in O(E + VLogV) time using Dijkstra’s algorithm. Can we do even better for Directed Acyclic Graph (DAG)?
We can calculate single source shortest distances in O(V+E) time for DAGs. The idea is to use Topological Sorting.'

When you have a DAG, try to topologically sort it, and so:
1. Topological sort the DAG. Path from u to v implies that u is before v in the ordering
2. One pass over vertices, relaxing each edge that leaves each vertex
(This does not depend on which vertex you choose to start with)
"""
def dfs(graph):
    color = {}
    parent = {}

    topo_stack = []

    for u in graph.keys():
        color[u] = 'white'
        parent[u] = None

    for u in graph.keys():
        if color[u] == 'white':
            dfs_visit(graph, u, color, parent, topo_stack)
    return topo_stack[::-1]

#edge (u, v) is a forward edge if discover[u] < discover[v]
# and a cross edge if discover[u] > discover[v]
def dfs_visit(graph, u, color, parent, topo_stack):
    color[u] = 'gray'

    for v in graph[u]:
        if color[v] == 'white':
            parent[v] = u
            print('tree edge {} --> {}'.format(u, v))
            dfs_visit(graph, v, color, parent, topo_stack)

    color[u] = 'black'
    topo_stack.append(u)


def dag_shortest_path(graph, start):
    topologically_sorted_vertices = dfs(graph)
    d = {}
    p = {}
    for u in graph:
        d[u] = math.inf
        p[u] = None
    d[start] = 0

    for u in topologically_sorted_vertices:
        for v in graph[u]:
            if d[v] > d[u] + graph[u][v]:
                d[v] = d[u] + graph[u][v]
                p[v] = u

    return d, p

g = {'S': {'A': 10, 'E': 8}, 'A': {'C': 2}, 'B': {'A': 1}, 'E': {'D': 1}, 'D': {'C': -1, 'A': -4}, 'C': {'B': -2}}
d, p = dag_shortest_path(g, 'S')
print(d)
print(p)