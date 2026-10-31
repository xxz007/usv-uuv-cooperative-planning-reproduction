"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P3/P3-L1/源码/p3_l1_communication.py
"""
import math
import networkx as nx



def communication_graph(usv, positions, radius):
    graph = nx.Graph()
    graph.add_node('USV', position_m=usv)
    distances = {}
    for boat, position in positions.items():
        graph.add_node(boat, position_m=position)
        distance = math.dist(usv, position)
        distances[boat] = distance
        if distance <= radius:
            graph.add_edge('USV', boat, distance_m=distance)
    return graph, distances

