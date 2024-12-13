import numpy

class Vertex:
    def __init__(self, name):
        self.name = name
        self.neighbors = []
        self.distance = numpy.inf
        self.visited = False
    
    def new_neighbor(self, neighbor_name, neighbor_cost):
        self.neighbors.append((neighbor_name, neighbor_cost))
    
    def display_vertex(self):
        neighbor_strings = []

        for neighbor in self.neighbors:
            neighbor_strings.append(f"{neighbor[0]} ({neighbor[1]})")

        return f"{self.name} -- [{', '.join(neighbor_strings)}]"

class Graph:
    def __init__(self):
        self.vertices = {}
    
    def new_vertex(self, name):
        if name not in self.vertices:
            self.vertices[name] = Vertex(name)
    
    def new_edge(self, starting_v_name, arrival_v_name, cost):
        if starting_v_name in self.vertices and arrival_v_name in self.vertices :
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

                if current_cost != numpy.inf and current_cost != 0:
                    next_vertice_name = vertices_names_list[cost_index]
                    self.new_edge(current_vertice_name, next_vertice_name, current_cost)