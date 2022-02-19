import itertools
import math
import os
import pickle
import heapq
import networkx as nx
from multiarm_planner.mrdrrt.robot_env import MultiRobotEnv

from multiarm_planner.profile_utils import timefunc

from .prm_planner import PRMPlanner
from .implicit_graph import ImplicitGraph, tensor_node_to_vector


class MRdRRTPlanner(object):
    """
    Multi-robot discrete RRT algorithm for coordinated centralized planning.

    Simple implementation of:

    Solovey, Kiril, Oren Salzman, and Dan Halperin.
    "Finding a needle in an exponential haystack: Discrete RRT for exploration
    of implicit roadmaps in multi-robot motion planning." Algorithmic Foundations
    of Robotics XI. Springer International Publishing, 2015. 591-607.
    """
    _EXPAND_N = 20
    _MAX_ITER = 5000

    def __init__(self, env: MultiRobotEnv, visualize=False):
        self.env = env
        self.implicit_graph = None # type: ImplicitGraph
        self.tree = nx.Graph()
        self.visualize = visualize

    def oracle(self, q_near, q_rand):
        """
        Direction oracle, as defined in Oren's paper.
        Given randomly sampled comp config and nearest config on current tree,
        return qnew, a neighbor of qnear on the implicit graph that hasn't been
        explored yet, and is closest (by sum of euclidean distances) to qnear.
        """
        q_new = self.implicit_graph.get_best_composite_neighbor(q_near, q_rand)
        return q_new

    def tree_nearest_neighbor(self, config):
        return min(self.tree.nodes, key=lambda node: self.env.composite_distance(node, config))

    def tree_k_nearest_neighbors(self, config, k):
        """
        Given composite configuration, find k closest ones in current tree.
        """
        neighbors = heapq.nsmallest(k, self.tree.nodes, key=lambda node: self.env.composite_distance(node, config))
        return neighbors
    
    @timefunc
    def expand(self, goal_configs):
        """
        Takes random samples and tries to expand tree in direction of sample.
        """
        for _ in range(self._EXPAND_N):
            q_rand = self.env.sample_free_multi_config()
            q_near = self.tree_nearest_neighbor(q_rand)
            q_new = self.oracle(q_near, q_rand)

            if q_new and q_new not in self.tree.nodes:
                self.tree.add_edge(q_near, q_new)
                if self.visualize:
                    self.implicit_graph.draw_composite_edge(q_near, q_new)
                if q_new == goal_configs:
                    break

    def local_connector(self, start, target):
        """
        Borrowed code from DiscoPygal's dRRT implementation.
        """
        num_robots = len(self.implicit_graph.roadmaps)
        paths = []
        for i, graph in enumerate(self.implicit_graph.roadmaps):
            if not nx.algorithms.has_path(graph, start[i], target[i]):
                return False
            path = nx.algorithms.shortest_path(graph, start[i], target[i], weight='weight')
            paths.append(path)
        
        # Generate the priority graph
        priority_graph = nx.DiGraph()
        priority_graph.add_nodes_from(range(num_robots))
        for i in range(num_robots):
            for j in range(num_robots):
                if i == j:
                    continue
                # Robot i stays in place in its target
                edge_i = [target[i]]

                # Robot j moves along its path
                for k in range(len(paths[j]) - 1):
                    edge_j = self.implicit_graph.get_single_roadmap_edge_path(j, paths[j][k], paths[j][k + 1])
                    if self.env.two_robots_collision_on_paths(i, edge_i, j, edge_j):
                        # If j collides with i when i is in its target, j needs to reach the target before i 
                        priority_graph.add_edge(j, i)

                # Robot i stays in place in its start
                edge_i = [start[i]]

                # Robot j moves along its path again
                for k in range(len(paths[j])-1):
                    edge_j = self.implicit_graph.get_single_roadmap_edge_path(j, paths[j][k], paths[j][k + 1])
                    if self.env.two_robots_collision_on_paths(i, edge_i, j, edge_j):
                        # If j collides with i when i is in its start, i needs to reach the target before j
                        priority_graph.add_edge(i, j)
        
        # If there are cycles, no ordering can be found
        if not nx.algorithms.is_directed_acyclic_graph(priority_graph):
            return False

        # Build a path based on the topological ordering
        curr_vertex = start
        for r in nx.algorithms.topological_sort(priority_graph):
            for v in paths[r]:
                # Advance only the correct robot
                new_ptr = [p for p in curr_vertex]
                new_ptr[r] = v
                new_ptr = tuple(new_ptr)
                self.tree.add_edge(curr_vertex, new_ptr)
                if self.visualize:
                    self.implicit_graph.draw_composite_edge(curr_vertex, new_ptr)
                curr_vertex = new_ptr
                
        assert(curr_vertex == target) # We should have reached the destination

        # # If managed to connect to the goal.
        # self.tree.add_edge(neighbor, goal_configs, path=path)
        # # if self.visualize:
        #     # self.env.draw_line_between_multi_configs(neighbor, goal_configs, path)
        # return True
        return True

    @timefunc
    def connect_to_target(self, goal_configs, iteration):
        """
        Check if it's possible to get to goal from closest nodes in current tree.
        Called at the end of each iteration.
        Input: list of goal configurations (goal composite config)
        """
        neighbors = self.tree_k_nearest_neighbors(goal_configs, int(math.log(iteration + 1, 2)))
        for neighbor in neighbors:
            if neighbor == goal_configs:
                # If 'expand' has already reached the target, no need to try to connect.
                return True
            success = self.local_connector(neighbor, goal_configs)
            if success:
                return True
        return False

    @timefunc
    def find_path(self, start_configs, goal_configs):
        """
        Main function for MRdRRT. Expands tree to find path from start to goal.
        Inputs: list of start and goal configs for robots.
        """
        if self.implicit_graph is None:
            print("Must create PRM graphs first for the implicit graph!"
            "Either run with mrdrrt.build_implicit_graph_with_prm or load from file with mrdrrt.load_implicit_graph_from_file")
            return
        assert len(start_configs) == len(goal_configs), "Start and goal configurations don't match in length"

        print("Looking for a path...")

        # Put start config in tree
        self.tree.add_node(start_configs)
        success = None

        for i in range(1, self._MAX_ITER + 1):
            self.expand(goal_configs)
            success = self.connect_to_target(goal_configs, i)
            if success:
                print("Found a path! Constructing final path now..")
                break
            if(i % 50 == 0):
                print(str(i) + "th iteration")

        if success:
            nodes = nx.shortest_path(self.tree, source=start_configs, target=goal_configs) # TODO add weights when inserting to tree
            paths_between_nodes = list([tensor_node_to_vector(node1)] + self.implicit_graph.create_movement_path_on_tensor_edge(node1, node2)
                                    for node1, node2 in zip(nodes, nodes[1:]))
            paths_between_nodes.append([tensor_node_to_vector(goal_configs)])
            path = list(itertools.chain.from_iterable(paths_between_nodes))
            return path
        else:
            print("Failed to find path - hit maximum iterations.")

    def task_cache_path(self, task_path):
        return os.path.splitext(task_path)[0] + "_cached.p"

    def load_implicit_graph_from_cache_file(self, task_path):
        pickle_path = self.task_cache_path(task_path)
        with open(pickle_path, 'rb') as f:
            prm_graphs = pickle.load(f)
            self.implicit_graph = ImplicitGraph(self.env, prm_graphs)

    def cache_loaded_graphs_to_file(self, task_path):
        pickle_path = self.task_cache_path(task_path)
        with open(pickle_path, "wb") as f:
            pickle.dump(self.implicit_graph.roadmaps, f)
        print("Saved roadmaps.")

    def generate_implicit_graph_with_prm(self, start_configs, goal_configs, **kwargs):
        prm_graphs = []
        for i in range(len(start_configs)):
            self.env.setup_single_prm(i, start_configs, goal_configs, **kwargs)
            prm_planner = PRMPlanner(self.env.robot_envs[i], n_nodes=150, visualize=False)
            prm_planner.generate_roadmap(start_configs[i], goal_configs[i])
            prm_graphs.append(prm_planner.graph)
        self.implicit_graph = ImplicitGraph(self.env, prm_graphs)

    def get_implicit_graph(self, start_configs, goal_configs, ur5_poses, cache_drrt, task_path):
        if cache_drrt:
            try:
                self.load_implicit_graph_from_cache_file(task_path)
            except:
                print("Can't load implicit graph from file.")
        if not self.implicit_graph:
            self.generate_implicit_graph_with_prm(start_configs, goal_configs, ur5_poses=ur5_poses)
        if cache_drrt:
            self.cache_loaded_graphs_to_file(task_path)


