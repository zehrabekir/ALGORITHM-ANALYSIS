import heapq

def read_matrices_from_file(file_path):
    matrices = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        current_matrix = []
        for line in lines:
            if line.strip():
                row = [float(value) for value in line.strip().split(':')]
                current_matrix.append(row)
            else:
                matrices.append(current_matrix)
                current_matrix = []
        if current_matrix:
            matrices.append(current_matrix)
    return matrices

def dijkstra(graph, start, end, bandwidth_constraint, delay_threshold, reliability_threshold):
    heap = [(0, start, [])]
    visited = set()

    while heap:
        (cost, node, path) = heapq.heappop(heap)
        if node not in visited:
            visited.add(node)
            path = path + [node]
            if node == end and cost <= bandwidth_constraint and cost <= delay_threshold and cost >= reliability_threshold:
                return path
            for neighbor, c, d, r in graph[node]:
                if neighbor not in visited:
                    heapq.heappush(heap, (cost + c, neighbor, path))
    
    return []

def bellman_ford(graph, start, end, bandwidth_constraint, delay_threshold, reliability_threshold):
    distance = {node: float('infinity') for node in graph}
    predecessor = {node: None for node in graph}
    distance[start] = 0
    
    for _ in range(len(graph) - 1):
        for node in graph:
            for neighbor, c, d, r in graph[node]:
                if distance[node] + c < distance[neighbor]:
                    distance[neighbor] = distance[node] + c
                    predecessor[neighbor] = node
    
    path = []
    current = end
    while current:
        path.insert(0, current)
        current = predecessor[current]

    return path

def a_star(graph, start, end, bandwidth_constraint, delay_threshold, reliability_threshold):
    heap = [(0, start, [])]
    visited = set()

    while heap:
        (cost, node, path) = heapq.heappop(heap)
        if node not in visited:
            visited.add(node)
            path = path + [node]
            if node == end and cost <= bandwidth_constraint and cost <= delay_threshold and cost >= reliability_threshold:
                return path
            for neighbor, c, d, r in graph[node]:
                if neighbor not in visited:
                    heapq.heappush(heap, (cost + c + heuristic(neighbor, end), neighbor, path))
    
    return []

def heuristic(node, end):

    return 0

def floyd_warshall(graph):
    dist = {v: {w: 0 if v == w else float('infinity') for w in graph} for v in graph}
    pred = {v: {w: None for w in graph} for v in graph}

    for node in graph:
        for neighbor, c, d, r in graph[node]:
            dist[node][neighbor] = c
            pred[node][neighbor] = node

    for k in graph:
        for i in graph:
            for j in graph:
                if dist[i][k] + dist[k][j] < dist[i][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
                    pred[i][j] = pred[k][j]

    return dist, pred

def reconstruct_path(start, end, pred):
    path = []
    current = end
    while current is not None:
        path.insert(0, current)
        current = pred[start][current]
    
    return path

def print_matrix(matrix):
    for i, row in enumerate(matrix):
        print(f"{i}: {' '.join(map(str, row))}")

def main():
    file_paths = ['1.txt', '2.txt'] 
    all_matrices = []
    for file_path in file_paths:
        matrices = read_matrices_from_file(file_path)
        all_matrices.extend(matrices)

    adjacency_matrix = all_matrices[0]
    bandwidth_matrix = all_matrices[1]
    delay_matrix = all_matrices[2]
    reliability_matrix = all_matrices[3]

    print("Adjacency Matrix:")
    print_matrix(adjacency_matrix)
    print("\nBandwidth Matrix:")
    print_matrix(bandwidth_matrix)
    print("\nDelay Matrix:")
    print_matrix(delay_matrix)
    print("\nReliability Matrix:")
    print_matrix(reliability_matrix)

    source_node = 1
    destination_node = 24
    bandwidth_constraint = 5
    delay_threshold = 40
    reliability_threshold = 0.70

    graph = {}
    for i in range(len(adjacency_matrix)):
        neighbors = []
        for j in range(len(adjacency_matrix[i])):
            if adjacency_matrix[i][j] == 1:
                neighbors.append((j + 1, bandwidth_matrix[i][j], delay_matrix[i][j], reliability_matrix[i][j]))
        graph[i + 1] = neighbors

    dijkstra_result = dijkstra(graph, source_node, destination_node, bandwidth_constraint, delay_threshold, reliability_threshold)
    print("\nDijkstra Result:")
    if dijkstra_result:
        print(' '.join(map(str, dijkstra_result)))
    else:
        print("No path found.")

    bellman_ford_result = bellman_ford(graph, source_node, destination_node, bandwidth_constraint, delay_threshold, reliability_threshold)
    print("\nBellman-Ford Result:")
    if bellman_ford_result:
        print(' '.join(map(str, bellman_ford_result)))
    else:
        print("No path found.")

    a_star_result = a_star(graph, source_node, destination_node, bandwidth_constraint, delay_threshold, reliability_threshold)
    print("\nA* Result:")
    if a_star_result:
        print(' '.join(map(str, a_star_result)))
    else:
        print("No path found.")

    dist, pred = floyd_warshall(graph)
    floyd_warshall_result = reconstruct_path(source_node, destination_node, pred)
    print("\nFloyd-Warshall Result:")
    if floyd_warshall_result:
        print(' '.join(map(str, floyd_warshall_result)))
    else:
        print("No path found.")

if __name__ == "__main__":
    main()
