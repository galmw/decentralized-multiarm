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
        self.nn_k = 5 # Nearest-neighbor K const

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


    def save_roadmap(self, file_path):
        """
        Save generated roadmap for future use, in pickle.
        """
        with open(file_path, "wb") as f:
            pickle.dump(self.graph, f)
            print("Saved roadmap.")

    def load_roadmap(self, file_path):
        """
        Loads pickle with pre-made roadmap.
        """
        print("Loading roadmap.")
        with open(file_path, 'rb') as f:
            self.graph = pickle.load(f)
    
    # def PlotRoadmap(self):
    #     """Plots roadmap's nodes and edges.
    #     Assumes that roadmap and pyplot figure are initialized.
    #     """
    #     print("Plotting roadmap..")
    #     for i, v in enumerate(self.graph.vertices):
    #         self.env.PlotPoint(v)
    #         for n in self.graph.edges[i]:
    #             self.env.PlotEdge(v, self.graph.vertices[n])

    # def VisualizePath(self, path, edgecolor='g-'):
    #     """Plots final path on roadmap between two points, in different color.
    #     Assumes pyplot figure is initialized.
    #     """
    #     self.env.PlotPoint(path[0], 'g', 7)
    #     self.env.PlotPoint(path[-1], 'm', 7)
    #     for i, config in enumerate(path[0:-1]):
    #         self.env.PlotEdge(path[i], path[i+1], edgecolor, 2)

    # def FindPath(self, sconfig, gconfig):
    #     """Find nearest vertices to sconfig and gconfig
    #     raw_input should be in numpy arrays of dim 3 (x,y,theta)
    #     """
    #     sid = self.graph.GetNearestNode(sconfig[0:2])
    #     gid = self.graph.GetNearestNode(gconfig[0:2])
    #     start_angle = sconfig[2]
    #     goal_angle = gconfig[2]

    #     point_path = self.graph.Djikstra(sid, gid)
    #     if len(point_path) == 0:
    #         return []

    #     # Add angles and sconfig, gconfig
    #     path = self.PostProcessPRMPath(point_path, sconfig, gconfig)

    #     if self.visualize:
    #         self.VisualizePath(point_path)
    #     return path
