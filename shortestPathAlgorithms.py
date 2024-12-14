import csv
import numpy
from graphObject import Graph

def extract_cost_matrix_csv():
    cost_matrix = []

    with open('graph.csv', newline='') as csvfile:
        costCSV = csv.reader(csvfile, delimiter=',')
        
        for row in costCSV:
            n_row = []

            for cost in row:
                cost = float(cost)
                if(cost == 1000000000000):
                    cost = float('inf')
                n_row.append(cost)
            cost_matrix.append(n_row)

    return numpy.array(cost_matrix)

def create_graph(cost_matrix, vertices_names):
    graph = Graph()

    # Add vetrices
    for vertice_index in range(len(cost_matrix)):
        graph.new_vertex(vertices_names[vertice_index])

    # add edges 
    graph.initiate_edges(cost_matrix)

    return graph

def main():
    vertices_names = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 
                      'I', 'J']
    
    cost_matrix = extract_cost_matrix_csv()
    graph = create_graph(cost_matrix, vertices_names)
    dists = graph.dijkstra_line('F')
    print(dists)

main()