import pickle
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
        self.nn_k = 15 # Nearest-neighbor K const

    def generate_roadmap(self, start, goal):
        """
        Standard PRM algorithm.
        """

        print("Generating roadmap...")
        self.graph.add_nodes_from([tuple(start), tuple(goal)])

        # Sample landmarks
        for _ in range(self.n_nodes):
            self.graph.add_node(self.env.sample_free_config())
        print(self.n_nodes, "landmarks sampled")

        # sklearn (which we use for nearest neighbor search) works with numpy array of points represented as numpy arrays
        points = list(p for p in self.graph.nodes)
        _points = np.array([np.array(p) for p in points])
        nearest_neighbors = sklearn.neighbors.NearestNeighbors(n_neighbors=self.nn_k, metric=self.env.distance, algorithm='auto')
        nearest_neighbors.fit(_points)

        # Try to connect neighbors
        print('Connecting landmarks')
        for i, node in enumerate(self.graph.nodes):
            # Obtain the K nearest neighbors
            k_neighbors = nearest_neighbors.kneighbors([_points[i]], return_distance=False)

            for j in k_neighbors[0]:
                neighbor = points[j]
                if node != neighbor and self.env.is_edge_collision_free(node, neighbor):
                    self.graph.add_edge(node, neighbor)
                    if self.visualize:
                        self.env.draw_line_between_configs(node, neighbor)
            if i % 100 == 0:
                print('Connected', i, 'landmarks to their nearest neighbors')
            i += 1
        
        if nx.has_path(self.graph, start, goal):
            print('PRM ran succesfully')
            return True
        else:
            print('PRM failed')
            return False

