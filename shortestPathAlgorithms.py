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
    dijkstraA = graph.dijkstra_line('A')
    print(f"dijkstra A : \n {dijkstraA}")

    dijkstraA = graph.dijkstra_line('G')
    print(f"dijkstra G : \n {dijkstraA}")

    dijkstra_matrix = graph.dijkstra_matrix()
    print(f"dijkstra Matrix : \n {dijkstra_matrix}")


    bellman_fordA = graph.bellman_ford_line('A')
    print(f"bellman_ford A : \n {bellman_fordA}")

    bellman_fordA = graph.bellman_ford_line('G')
    print(f"bellman_ford G : \n {bellman_fordA}")

    bellman_ford_matrix = graph.bellman_ford_matrix()
    print(f"bellman_ford Matrix : \n {bellman_ford_matrix}")

main()