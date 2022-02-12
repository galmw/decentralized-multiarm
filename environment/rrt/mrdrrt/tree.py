class Tree(object):
    """
    Tree structure, meant for use with implicit graph in MRdRRT.
    """
    def __init__(self, env, implicit_graph):
        self.nodes = []
        self.env = env
        self.implicit_graph = implicit_graph

    def add_node(self, config, parent=None, visualize=False):
        """
        Add vertex to tree.
        """
        node = TreeNode(config, parent=parent)
        self.nodes.append(node)
        if visualize:
            self.env.draw_line_between_multi_configs(parent.config, node.config)
        return node

    def nearest_neighbor(self, config):
        """
        Given composite configuration, find closest one in current tree.
        """
        min_dist = float("inf")
        nearest = None

        for node in self.nodes:
            dist = self.env.composite_distance(node.config, config)
            if (dist < min_dist):
                min_dist = dist
                nearest = node

        return nearest


class TreeNode(object):
    def __init__(self, config, parent=None):
        # configuration
        self.config = config
        # parent configuration
        self.parent = parent

    def retrace(self):
        """
        Get a list of nodes from start to itself
        :return: a list of nodes
        """
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node.parent
        return sequence[::-1]

    def __str__(self):
        return 'TreeNode(' + str(self.config) + ')'

    __repr__ = __str__
