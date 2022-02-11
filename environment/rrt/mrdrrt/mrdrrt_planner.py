import random
import os
import pickle

from environment.rrt.mrdrrt.prm_planner import PRMPlanner
from .tree import Tree, TreeNode
from .implicit_graph import ImplicitGraph
from .profile_utils import timefunc


class MRdRRTPlanner(object):
    """
    Multi-robot discrete RRT algorithm for coordinated centralized planning.

    Simple implementation of:

    Solovey, Kiril, Oren Salzman, and Dan Halperin.
    "Finding a needle in an exponential haystack: Discrete RRT for exploration
    of implicit roadmaps in multi-robot motion planning." Algorithmic Foundations
    of Robotics XI. Springer International Publishing, 2015. 591-607.
    """

    def __init__(self, env, visualize=False):
        self.env = env
        self.implicit_graph = None
        self.tree = Tree(self.env, self.implicit_graph)
        self.max_iter = 5000
        self.visualize = visualize

    #@timefunc
    def oracle(self, q_near, q_rand):
        """
        Direction oracle, as defined in Oren's paper.
        Given randomly sampled comp config and nearest config on current tree,
        return qnew, a neighbor of qnear on the implicit graph that hasn't been
        explored yet, and is closest (by sum of euclidean distances) to qnear.
        """
        q_new = self.implicit_graph.get_best_composite_neighbor(q_near, q_rand)
        return q_new

    #@timefunc
    def expand(self, goal_configs):
        """
        Takes random sample and tries to expand tree in direction of sample.
        """
        if random.random() > 0.3:
            q_rand = self.env.sample_free_multi_config()
        else:
            q_rand = goal_configs
        q_near = self.tree.nearest_neighbor(q_rand)
        q_new = self.oracle(q_near.config, q_rand)

        if self.env.is_edge_collision_free(q_near.config, q_new):
            self.tree.add_node(q_new, parent=q_near)

    @timefunc
    def connect_to_target(self, goal_configs):
        """
        Check if it's possible to get to goal from closest nodes in current tree.
        Called at the end of each iteration.
        Input: list of goal configurations (goal composite config)
        """

        neighbor = self.tree.nearest_neighbor(goal_configs)
        success = self.env.is_edge_collision_free(neighbor.config, goal_configs)
        if success:
            return self.tree.add_node(goal_configs, parent=neighbor)
        return None

    def find_path(self, start_configs, goal_configs):
        """
        Main function for MRdRRT. Expands tree to find path from start to goal.
        Inputs: list of start and goal configs for robots.
        """
        if self.implicit_graph is None:
            print("Must create PRM graphs first for the implicit graph!"
            "Either run with mrdrrt.build_implicit_graph_with_prm or load from file with mrdrrt.load_implicit_graph_from_file")
            return
        if len(start_configs) != len(goal_configs):
            print("Start and goal configurations don't match in length")
            return

        print("Looking for a path...")

        # Put start config in tree
        self.tree.add_node([tuple(conf) for conf in start_configs])
        success = None

        for i in range(self.max_iter):
            self.expand(goal_configs)
            success = self.connect_to_target(goal_configs)
            if success:
                print("Found a path! Constructing final path now..")
                path_nodes = success.retrace()
                break
            if(i % 50 == 0):
                print(str(i) + "th iteration")

        if success is None:
            print("Failed to find path - hit maximum iterations.")
        else:
            return [node.config for node in path_nodes]

    def task_cache_path(self, task_path):
        return os.path.splitext(task_path)[0] + "_cached.p"

    def load_implicit_graph_from_file(self, task_path):
        pickle_path = self.task_cache_path(task_path)
        try:
            with open(pickle_path, 'rb') as f:
                prm_graphs = pickle.load(f)
                self.implicit_graph = ImplicitGraph(self.env, prm_graphs)
        except:
            print("Can't load implicit graph from file.")

    def cache_loaded_graphs(self, task_path):
        pickle_path = self.task_cache_path(task_path)
        with open(pickle_path, "wb") as f:
            pickle.dump(self.implicit_graph.prm_graphs, f)
        print("Saved roadmaps.")

    def generate_implicit_graph_with_prm(self, start_configs, goal_configs, **kwargs):
        prm_graphs = []
        for i in range(len(start_configs)):
            self.env.setup_single_prm(i, start_configs, goal_configs, **kwargs)
            prm_planner = PRMPlanner(self.env.robot_envs[i], n_nodes=50, visualize=self.visualize)
            prm_planner.generate_roadmap(start_configs[i], goal_configs[i])
            prm_graphs.append(prm_planner.graph)
        self.implicit_graph = ImplicitGraph(self.env, prm_graphs)

    


