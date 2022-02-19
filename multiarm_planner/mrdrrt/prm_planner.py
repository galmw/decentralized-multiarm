import heapq
import numpy as np
import networkx as nx
import sklearn.neighbors


class PRMPlanner(object):
    """
    PRM planner node: interfaces with environment and graph structure.
    Can either generate a new roadmap or load a saved one.
    """

    def __init__(self, env, n_nodes=300, visualize=True):
        self.env = env
        self.graph = nx.Graph()
        self.n_nodes = n_nodes
        self.visualize = visualize
        self.nn_k = 6 # Nearest-neighbor K const

    def generate_roadmap(self, start, goal):
        """
        Standard PRM algorithm.
        """

        print(f"Generating roadmap with initial {self.n_nodes} nodes...")
        self.graph.add_nodes_from([tuple(start), tuple(goal)])

        # Sample landmarks
        for _ in range(self.n_nodes):
            self.graph.add_node(self.env.sample_free_config())
        print(self.n_nodes, "landmarks sampled")

        # sklearn (which we use for nearest neighbor search) works with numpy array of points represented as numpy arrays
        points = list(self.graph.nodes)
        _points = np.array([np.array(p) for p in points])
        nearest_neighbors = sklearn.neighbors.NearestNeighbors(n_neighbors=self.nn_k, metric=self.env.distance, algorithm='auto')
        nearest_neighbors.fit(_points)

        # Try to connect neighbors
        print('Connecting landmarks')
        for i, node in enumerate(points):
            # Obtain the K nearest neighbors
            k_neighbors = nearest_neighbors.kneighbors([_points[i]], return_distance=False)[0]
            for j in k_neighbors:
                neighbor = points[j]
                if node != neighbor:
                    assert i != j # Verify no loops
                    path = self.env.check_path_collision_free(node, neighbor)
                    if path:
                        self.graph.add_edge(node, neighbor, weight=self.env.distance(node, neighbor), path=path, path_start=node)
                        if self.visualize:
                            self.env.draw_line_between_configs(node, neighbor, path=path)

            if i % 50 == 0:
                print('Connected', i, 'landmarks to their nearest neighbors')
            i += 1
        
        while not nx.has_path(self.graph, start, goal):
            print("No valid path in PRM. Adding more samples")
            self.grow(50)
        return True

    def grow(self, num_samples):
        """
        Add samples to existing PRM. Slower than generating from scrath
        """
        new_nodes = [self.env.sample_free_config() for _ in range(num_samples)]
        print(num_samples, "more landmarks sampled")
        map(self.graph.add_node, new_nodes)

        for node in new_nodes:
            neighbors = heapq.nsmallest(self.nn_k, self.graph.nodes, key=lambda q: self.env.distance(node, q))
            for neighbor in neighbors:
                if neighbor != node:
                    path = self.env.check_path_collision_free(node, neighbor)
                    if path:
                        self.graph.add_edge(node, neighbor, weight=self.env.distance(node, neighbor), path=path, path_start=node)
                        if self.visualize:
                            self.env.draw_line_between_configs(node, neighbor, path=path)
        print(f"Grew PRM by {num_samples} nodes")
