import numpy as np
import itertools


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


def tensor_node_to_vector(q):
    return tuple(itertools.chain.from_iterable(q))


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
        paths = self.get_tensor_edge_paths(q, neighbor)
        for i in range(len(self.roadmaps)):
            for j in range(i + 1, len(self.roadmaps)):
                # Checking inner paths should suffice?
                if self.env.two_robots_collision_on_paths(i, paths[i], j, paths[j]):
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

    def draw_composite_edge(self, p, q):
        movement_path = self.create_movement_path_on_tensor_edge(p, q)
        self.env.draw_line_between_multi_configs(p, q, path=movement_path)

    def get_tensor_edge_paths(self, p, q):
        paths = []
        for i, roadmap in enumerate(self.roadmaps):
            if p[i] == q[i]:
                paths.append([p[i]])
            else:
                edge_i = roadmap.edges[p[i], q[i]]
                path = edge_i['path']
                if edge_i['path_start'] == q[i]:
                    path = path[::-1]
                paths.append(path)
        return paths
    
    def create_movement_path_on_tensor_edge(self, p, q):
        """
        Take two points on the tensor roadmap 
        """
        # If a robot stays in place there is no need to find its relevent path.
        paths = self.get_tensor_edge_paths(p, q)
        path = []
        for point in itertools.zip_longest(*paths):
            # zip_longest pads each path with 'None'. We want to put in there the last point in the path instead.
            path_point = [point[i] if point[i] else paths[i][-1] for i in range(len(self.roadmaps))]
            path.append(tensor_node_to_vector(path_point))
        return path
    

        
            
