class ImplicitGraph(object):
    """
    Defines implicit graph (composite of PRM roadmaps) for MRdRRT.
    """

    def __init__(self, env, prm_graphs):
        """
        Loads PRM roadmap that will define implicit graph.
        """
        self.prm_graphs = prm_graphs
        self.env = env

    def get_best_neighbor_in_individual_graph(self, index, q_near, q_rand):
        """
        Note: "Best" in dRRT is supposed to mean "with best angle". Here I am just doing "closest" for now.
        """
        graph = self.prm_graphs[index]
        graph_env = self.env.robot_envs[index]
        node = q_near[index]
        direction = q_rand[index]
        min_dist = float("inf")
        best = None

        for neighbor in graph.neighbors(node):
            dist = graph_env.distance(neighbor, direction)
            if (dist < min_dist):
                min_dist = dist
                best = neighbor

        return best
        
    def get_best_composite_neighbor(self, q_near, q_rand):
        """
        Given config on current tree and randomly sampled comp config,
        find neighbor of qnear that is best headed towards qrand
        """
        best = [self.get_best_neighbor_in_individual_graph(i, q_near, q_rand) for i in range(len(self.prm_graphs))]
        return best
