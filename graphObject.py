import numpy

class Vertex:
    def __init__(self, name):
        self.name = name
        self.neighbors = []
        self.distance = float('inf')
        self.visited = False

    def new_neighbor(self, neighbor_name, neighbor_cost):
        self.neighbors.append((neighbor_name, neighbor_cost))

    def display_vertex(self):
        neighbors_infos = []

        for neighbor in self.neighbors:
            neighbor_infos_string = f"{neighbor[0]} ({neighbor[1]})"
            neighbors_infos.append(neighbor_infos_string)

        neighbors_display = ", ".join(neighbors_infos)

        return f"{self.name} -- [{neighbors_display}]"

class Graph:
    def __init__(self):
        self.vertices = {}

    def new_vertex(self, name):
        if name not in self.vertices:
            self.vertices[name] = Vertex(name)

    def new_edge(self, starting_v_name, arrival_v_name, cost):
        if starting_v_name in self.vertices and arrival_v_name in self.vertices:
            self.vertices[starting_v_name].new_neighbor(arrival_v_name, cost)

    def display_graph(self):
        for vertex in self.vertices.values():
            print(vertex.display_vertex())

    def get_vertices_names(self):
        return list(self.vertices.keys())

    def initiate_edges(self, cost_matrix):
        vertices_names_list = self.get_vertices_names()

        for vertice_index in range(len(cost_matrix)):
            current_vertice_name = vertices_names_list[vertice_index]

            for cost_index in range(len(cost_matrix[0])):
                current_cost = cost_matrix[vertice_index][cost_index]
                
                if current_cost != float('inf') and current_cost != 0:
                    next_vertice_name = vertices_names_list[cost_index]
                    self.new_edge(current_vertice_name, next_vertice_name, current_cost)

    def get_unvisited_neighbors(self, vertex_name):
        unvisited_neighbors = []

        for neighbor_name, neighbor_cost in self.vertices[vertex_name].neighbors:
            neighbor = self.vertices[neighbor_name]

            if not(neighbor.visited):
                unvisited_neighbors.append((neighbor, neighbor_cost))

        return unvisited_neighbors

    def dijkstra_line(self, starting_vertex_name, display = False):
        distances_line = []

        self.vertices[starting_vertex_name].distance = 0
        unvisited_vertices = list(self.vertices.values())

        while len(unvisited_vertices) > 0:
            # 1. Selction current_vertex (smallest distance)
            current_vertex = min(unvisited_vertices, key=lambda vertex: vertex.distance)
            
            # no cooncection, stop
            if current_vertex.distance == float('inf'):
                break
            
            # current is visited
            current_vertex.visited = True
            unvisited_vertices.remove(current_vertex)

            # 2. distances maj
            unvisited_neighbors = self.get_unvisited_neighbors(current_vertex.name)

            for neighbor_vertex, neighbor_cost in unvisited_neighbors:
                new_distance = current_vertex.distance + neighbor_cost

                # if better distance --> maj (plus petite)
                if new_distance < neighbor_vertex.distance:
                    neighbor_vertex.distance = new_distance

        for vertex in self.vertices.values():
            if(display):
                print(f"Distance from {starting_vertex_name} to {vertex.name} : {vertex.distance}")
            distances_line.append(float(vertex.distance))
            # reset
            vertex.distance = float('inf')
            vertex.visited = False
        

        return distances_line

    def dijkstra_matrix(self):
        distances_matrix = []
        
        for vertex in self.vertices.values():
            current_starting_vertex_name = vertex.name
            current_line = self.dijkstra_line(current_starting_vertex_name)
            distances_matrix.append(current_line)

        return numpy.array(distances_matrix)
    
     # Bellman-Ford Algorithm
    def bellman_ford_line(self, start_vertex_name, display=False):
        vertices_names = self.get_vertices_names()
        n = len(vertices_names)
        distances_line = [float('inf')] * n

        # Set the start vertex distance to 0
        start_index = vertices_names.index(start_vertex_name)
        distances_line[start_index] = 0

        # Relax edges (n-1) times
        for n in range(n - 1):
            for vertex in self.vertices.values():
                for neighbor_name, neighbor_cost in vertex.neighbors:
                    u_index = vertices_names.index(vertex.name)
                    v_index = vertices_names.index(neighbor_name)
                    if distances_line[u_index] + neighbor_cost < distances_line[v_index]:
                        distances_line[v_index] = distances_line[u_index] + neighbor_cost

        # Check for negative weight cycles
        for vertex in self.vertices.values():
            for neighbor_name, neighbor_cost in vertex.neighbors:
                u_index = vertices_names.index(vertex.name)
                v_index = vertices_names.index(neighbor_name)
                if distances_line[u_index] + neighbor_cost < distances_line[v_index]:
                    print("Graph contains a negative weight cycle")

        if display:
            for i, distance in enumerate(distances_line):
                print(f"Distance from {start_vertex_name} to {vertices_names[i]}: {distance}")

        return distances_line

    def bellman_ford_matrix(self, display=False):
        distances_matrix = []
        for vertex in self.vertices.values():
            distances_line = self.bellman_ford_line(vertex.name, display=False)
            distances_matrix.append(distances_line)

        distances_matrix = numpy.array(distances_matrix)

        return distances_matrix
    
"""
1. le couurant vertex --> le plus petit, marche aussi pour permier car vertex.distance tous à inf
2. Reg les voisins de courrant vertex, comparer les dist, si meilleures (new < connue pour ce stade), mettre à jour
--> reset les vertrices

sources
1. vidéos cours,
2. https://en.wikipedia.org/wiki/Dijkstra's_algorithm,
3. cours algo première
"""