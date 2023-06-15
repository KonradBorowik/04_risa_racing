import networkx as nx


class PathPlanner:
    def __init__(self, grid, resolution, origin) -> None:
        self.grid = grid
        self.resolution = resolution
        self.start = (int(self.grid.shape[0] - origin[1]*-1 / self.resolution), int(self.grid.shape[1] - origin[0]*-1 / self.resolution))
        self.first_checkpoint = ()
        self.second_checkpoint = ()
        # self.stop = (self.grid.shape[0] - origin[1]*-1 / self.resolution, self.grid.shape[1] - origin[0]*-1 / self.resolution)
        
    def calculate_path(self):
        rows, cols = self.grid.shape
        print(f'start point {self.start}') # for debugging purposes
        # Create a graph from the road network
        graph = nx.Graph()
        added_nodes = 0
        for i in range(rows):
            for j in range(cols):
                if self.grid[i, j] == 0:  # Free space
                    if (i, j) == (self.start):
                        print(f'aaa {added_nodes}') # for debugging purposes
                    x = j
                    y = i
                    graph.add_node((x, y))
                    added_nodes += 1

        print(f'added nodes = {added_nodes}') # for debugging purposes

         # Add edges to the graph
        added_edges = 0
        for node in graph.nodes():
            x, y = node
            neighbors = [(x -1, y), (x, y -1), (x+1, y), (x, y+1)]
            for neighbor in neighbors:
                if neighbor in graph.nodes():
                    graph.add_edge(node, neighbor)
                    added_edges += 1
        print(f'added edges {added_edges}') # for debugging purposes
        # Calculate the shortest path using Dijkstra's algorithm
        print(f'start point in graph {self.grid[self.start[0]][self.start[1]]}') # for debugging purposes
        
        path = nx.dijkstra_path(graph, graph[132323], graph[132323])
        # path.extend(nx.dijkstra_path(graph, self.first_checkpoint, self.second_checkpoint))
        # path.extend(nx.dijkstra_path(graph, self.second_checkpoint, self.start))

        return path
