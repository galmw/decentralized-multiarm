import numpy as np


def unit_vector(vector):
    """
    Returns the unit vector of the vector.
    """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """
    Returns the angle in radians between vectors 'v1' and 'v2'::
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


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
        Note: "Best" in dRRT is supposed to mean "with best angle".
        """
        graph = self.prm_graphs[index]
        graph_env = self.env.robot_envs[index]
        target_vector = graph_env.difference(q_rand, q_near)
        min_angle = float("inf")
        best = None

        for neighbor in graph.neighbors(q_near):
            vector = graph_env.difference(neighbor, q_near)
            angle = angle_between(target_vector, vector)
            if (angle < min_angle):
                min_angle = angle
                best = neighbor

        return best
        
    def get_best_composite_neighbor(self, q_near, q_rand):
        """
        Given config on current tree and randomly sampled comp config,
        find neighbor of qnear that is best headed towards qrand
        """
        best = [self.get_best_neighbor_in_individual_graph(i, q_near[i], q_rand[i]) for i in range(len(self.prm_graphs))]
        return best

