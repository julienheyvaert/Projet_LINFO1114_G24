import csv
import numpy

def main():
    cost_matrix = []

    with open('graph.csv', newline='') as csvfile:
        costCSV = csv.reader(csvfile, delimiter=',')
        
        for row in costCSV:
            n_row = []
            for v in row:
                if(v == 'inf'):
                    v = numpy.inf
                n_row.append(float(v))
            cost_matrix.append(n_row)

    cost_matrix = numpy.array(cost_matrix)

    print(cost_matrix)

main()