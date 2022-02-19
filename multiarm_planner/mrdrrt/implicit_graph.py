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

    def __init__(self, env, roadmaps):
        """
        Loads PRM roadmap that will define implicit graph.
        """
        self.roadmaps = roadmaps
        self.env = env

    def _get_best_neighbor_in_individual_graph(self, index, q_near, q_rand):
        """
        Note: "Best" in dRRT is supposed to mean "with best angle".
        """
        graph = self.roadmaps[index]
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
        
    def clean_neighbor_movement(self, q, neighbor):
        if q == neighbor:
            return None
        for i in range(len(self.roadmaps)):
            for j in range(i + 1, len(self.roadmaps)):
                path_i = self.roadmaps[i].edges[q[i], neighbor[i]]['path']
                path_j = self.roadmaps[j].edges[q[j], neighbor[j]]['path']
                if self.env.two_robots_collision_on_paths(i, path_i, j, path_j):
                    # If they collide, put robot j in place and try again
                    new_neighbor = tuple(vertex if idx != j else q[idx] for idx, vertex in enumerate(neighbor))
                    return self.clean_tensor_edge(q, new_neighbor)
        return neighbor

    def get_best_composite_neighbor(self, q_near, q_rand, clean_movement=True):
        """
        Given config on current tree and randomly sampled comp config,
        find neighbor of qnear that is best headed towards qrand
        """
        best_neighbor = tuple(self._get_best_neighbor_in_individual_graph(i, q_near[i], q_rand[i]) for i in range(len(self.roadmaps)))
        if clean_movement:
            best_neighbor = self.clean_neighbor_movement(q_near, best_neighbor)
        return best_neighbor


