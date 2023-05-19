import matplotlib.pyplot as plt
import networkx as nx
import heapq

# Define a function to compute the shortest path using Dijkstra's algorithm


def dijkstra(G, source):
    # Initialize the distance and predecessor dictionaries
    dist = {node: float('inf') for node in G.nodes()}
    pred = {node: None for node in G.nodes()}

    # Set the distance of the source node to 0
    dist[source] = 0

    # Initialize a priority queue with the source node
    pq = [(0, source)]

    # Loop until the priority queue is empty
    while pq:
        # Get the node with the smallest distance
        d, u = heapq.heappop(pq)

        # Skip the node if its distance has already been finalized
        if d > dist[u]:
            continue

        # Loop through the neighbors of the node
        for v, w in G[u].items():
            # Compute the tentative distance to the neighbor
            alt = dist[u] + w['weight']

            # Update the distance and predecessor if the tentative distance is smaller
            if alt < dist[v]:
                dist[v] = alt
                pred[v] = u

                # Add the neighbor to the priority queue
                heapq.heappush(pq, (alt, v))

    # Return the distance and predecessor dictionaries
    return dist, pred


# Read the input data
with open('input.txt', 'r') as f:
    # Read the number of nodes and edges
    n, m = map(int, f.readline().strip().split(','))

    # Read the edges
    edges = [tuple(line.strip().split(',')) for line in f]

# Create a new empty graph
G = nx.Graph()

# Loop through the edges and add them to the graph
for src, dest, weight in edges:
    G.add_edge(src, dest, weight=int(weight))
    G.add_edge(dest, src, weight=int(weight))  # since the graph is undirected

# Compute the shortest path from each node using Dijkstra's algorithm
forwarding_tables = {}
for src in G.nodes():
    # Compute the shortest path from the source node to every other node in the graph
    dist, pred = dijkstra(G, src)

    # Construct the forwarding table for the source node
    forwarding_table = {}
    for dest in G.nodes():
        if dest == src:
            continue

        # Initialize the path with the destination node
        path = [dest]

        # Follow the predecessor pointers to the source node
        while pred[path[-1]] is not None:
            path.append(pred[path[-1]])
        path.reverse()

        # Store the path and the next hop in the forwarding table
        forwarding_table[dest] = (path, path[1])

    # Store the forwarding table for the source node
    forwarding_tables[src] = forwarding_table

# Print the forwarding tables
for src, table in forwarding_tables.items():
    print(f"Forwarding table for node {src}:")
    for dest, (path, next_hop) in table.items():
        print(
            f"  {dest}: {' -> '.join(str(node) for node in path)} (next hop: {next_hop})")

# Draw the graph
pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True)

# Draw the edge labels
edge_labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

# Show the graph
plt.show()
