import sys
import math
from collections import deque

class Graph:
    def __init__(self):
        self.vertices = None

    class Vertex:
        def __init__(self, id):
            self.id = id
            self.edges = None
            self.next = None
            self.encountered = False
            self.done = False
            self.parent = None
            self.cost = float('inf')

        def reinit(self):
            self.done = False
            self.encountered = False
            self.parent = None
            self.cost = float('inf')

        def add_to_adjacency_list(self, edge):
            edge.next = self.edges
            self.edges = edge

        def path_string(self):
            if self.parent is None:
                return self.id
            return self.parent.path_string() + " -> " + self.id

        def __str__(self):
            result = f"vertex {self.id}:\n"
            edge = self.edges
            while edge:
                result += f"\tedge to {edge.end.id} (cost = {edge.cost})\n"
                edge = edge.next
            return result

    class Edge:
        def __init__(self, start, end, cost):
            self.start = start
            self.end = end
            self.cost = cost
            self.next = None

    def reinit_vertices(self):
        vertex = self.vertices
        while vertex:
            vertex.reinit()
            vertex = vertex.next

    def get_vertex(self, id):
        vertex = self.vertices
        while vertex and vertex.id != id:
            vertex = vertex.next
        return vertex

    def add_vertex(self, id):
        vertex = self.Vertex(id)
        vertex.next = self.vertices
        self.vertices = vertex
        return vertex

    def add_edge(self, start_vertex_id, end_vertex_id, cost):
        start = self.get_vertex(start_vertex_id)
        if start is None:
            start = self.add_vertex(start_vertex_id)

        end = self.get_vertex(end_vertex_id)
        if end is None:
            end = self.add_vertex(end_vertex_id)

        edge = self.Edge(start, end, cost)
        start.add_to_adjacency_list(edge)

    def __str__(self):
        result = ""
        vertex = self.vertices
        while vertex:
            result += str(vertex)
            vertex = vertex.next
        return result

    def init_from_file(self, file_name):
        try:
            with open(file_name, 'r') as file:
                for line in file:
                    parts = line.split()
                    start_id = parts[0]
                    end_id = parts[1]
                    cost = float(parts[2])
                    self.add_edge(start_id, end_id, cost)
        except IOError:
            print(f"Error accessing {file_name}")
            sys.exit(1)
        except (IndexError, ValueError):
            print(f"Invalid input line: {line}")
            sys.exit(1)

    def depth_first_trav(self, origin_id):
        self.reinit_vertices()
        start = self.get_vertex(origin_id)
        if start is None:
            raise ValueError(f"No such vertex: {origin_id}")
        Graph._df_trav(start, None)

    def _df_trav(v, parent):
        print(v.id)
        v.done = True
        v.parent = parent

        edge = v.edges
        while edge:
            if not edge.end.done:
                Graph._df_trav(edge.end, v)
            edge = edge.next

    def breadth_first_trav(self, origin_id):
        self.reinit_vertices()
        origin = self.get_vertex(origin_id)
        if origin is None:
            raise ValueError(f"No such vertex: {origin_id}")
        Graph._bf_trav(origin)

    def _bf_trav(origin):
        origin.encountered = True
        origin.parent = None
        queue = deque([origin])

        while queue:
            v = queue.popleft()
            print(v.id)
            edge = v.edges
            while edge:
                if not edge.end.encountered:
                    edge.end.encountered = True
                    edge.end.parent = v
                    queue.append(edge.end)
                edge = edge.next

    def prim(self, origin_id):
        self.reinit_vertices()
        origin = self.get_vertex(origin_id)
        if origin is None:
            raise ValueError(f"No such vertex: {origin_id}")
        origin.done = True

        while True:
            edge_to_add = None
            v = self.vertices
            while v:
                if v.done:
                    edge = v.edges
                    while edge:
                        if not edge.end.done and (edge_to_add is None or edge.cost < edge_to_add.cost):
                            edge_to_add = edge
                        edge = edge.next
                v = v.next

            if edge_to_add is None:
                return

            print(f"\tadding edge ({edge_to_add.start.id}, {edge_to_add.end.id})")
            edge_to_add.end.parent = edge_to_add.start
            edge_to_add.end.done = True

    def dijkstra(self, origin_id):
        self.reinit_vertices()
        origin = self.get_vertex(origin_id)
        if origin is None:
            raise ValueError(f"No such vertex: {origin_id}")
        origin.cost = 0

        while True:
            w = None
            v = self.vertices
            while v:
                if not v.done and (w is None or v.cost < w.cost):
                    w = v
                v = v.next

            if w is None or w.cost == float('inf'):
                return

            print(f"\tfinalizing {w.id} (cost = {w.cost}" +
                  (f", parent = {w.parent.id})" if w.parent else ")"))
            print(f"\t\tpath = {w.path_string()}")
            w.done = True

            edge = w.edges
            while edge:
                x = edge.end
                if not x.done:
                    cost_via_w = w.cost + edge.cost
                    if cost_via_w < x.cost:
                        x.cost = cost_via_w
                        x.parent = w
                edge = edge.next

    # Task 1
    def df_trav_abuja():
        return []

    # Task 2
    def bf_trav_abuja():
        return []

    # Task 3
    def is_connected(self, start, end):
        return False

    # Task 4
    def is_bipartite(self):
        return False

if __name__ == "__main__":
    g = Graph()

    file_name = input("Graph info file: ")
    g.init_from_file(file_name)

    start = input("starting point: ")

    print(f"depth-first traversal from {start}:")
    g.depth_first_trav(start)

    print(f"breadth-first traversal from {start}:")
    g.breadth_first_trav(start)

    print(f"Prim's algorithm from {start}:")
    g.prim(start)

    print(f"Dijkstra's algorithm from {start}:")
    g.dijkstra(start)
