import numpy as np
import networkx as nx

def read_matrix_from_file(file_path):
    """Reads a matrix from a file and returns it as a NumPy array."""
    with open(file_path, 'r') as file:
        matrix = []
        for line in file:
            row = [float(value) for value in line.strip().split(':')]
            matrix.append(row)
        return np.array(matrix)

def check_matrix_properties(matrix, min_value, max_value):
    """Checks if matrix values are within the specified range."""
    for row in matrix:
        for value in row:
            if not (min_value <= value <= max_value):
                return False
    return True

def calculate_degree_centrality(adjacency_matrix):
    """Calculates the degree centrality of each node in a graph."""
    degree_centrality = np.sum(adjacency_matrix, axis=1)
    return degree_centrality

def dijkstra_algorithm(graph, source):
    """Runs Dijkstra's algorithm on a graph and returns the shortest paths."""
    return nx.single_source_dijkstra_path_length(graph, source)

def bellman_ford_algorithm(graph, source):
    """Runs Bellman-Ford algorithm on a graph and returns the shortest paths."""
    return nx.single_source_bellman_ford_path_length(graph, source)

def a_star_algorithm(graph, source, target, heuristic=None):
    """Runs A* algorithm on a graph and returns the shortest path."""
    return nx.astar_path_length(graph, source, target, heuristic=heuristic)

def floyd_warshall_algorithm(graph):
    """Runs Floyd-Warshall algorithm on a graph and returns the shortest paths."""
    return nx.floyd_warshall_numpy(graph)

def main():

    adjacency_matrix = read_matrix_from_file('USNET_AjdMatrix.txt')


    if not check_matrix_properties(adjacency_matrix, 0, 1):
        print("Error: Adjacency matrix values are not within the specified range.")
        return


    graph = nx.from_numpy_array(adjacency_matrix)

    degree_centrality = calculate_degree_centrality(adjacency_matrix)

    print("Degree Centrality:")
    for i, centrality in enumerate(degree_centrality):
        print(f"Node {i + 1}: Degree Centrality = {centrality}")

    source_node = 0
    target_node = 3

    dijkstra_result = dijkstra_algorithm(graph, source_node)
    print("\nDijkstra's Shortest Paths from Node", source_node, ":")
    for node, distance in dijkstra_result.items():
        print(f"  Node {node}: Distance = {distance}")

    bellman_ford_result = bellman_ford_algorithm(graph, source_node)
    print("\nBellman-Ford Shortest Paths from Node", source_node, ":")
    for node, distance in bellman_ford_result.items():
        print(f"  Node {node}: Distance = {distance}")

    a_star_result = a_star_algorithm(graph, source_node, target_node)
    print("\nA* Shortest Path from Node", source_node, "to Node", target_node, ":", a_star_result)

    floyd_warshall_result = floyd_warshall_algorithm(graph)
    print("\nFloyd-Warshall Shortest Paths:")
    print(np.array(floyd_warshall_result))

if __name__ == "__main__":
    main()
