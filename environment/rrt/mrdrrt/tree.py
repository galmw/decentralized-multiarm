class Tree(object):
    """
    Tree structure, meant for use with implicit graph in MRdRRT.
    """
    def __init__(self, env):
        self.nodes = []
        self.env = env

    def add_node(self, config, parent=None, path=[], visualize=False):
        """
        Add vertex to tree.
        """
        node = TreeNode(config, parent=parent, path=path)
        self.nodes.append(node)
        if visualize:
            self.env.draw_line_between_multi_configs(parent.config, node.config, path=path)
        return node




class TreeNode(object):
    def __init__(self, config, parent=None, path=[]):
        # configuration
        self.config = config
        # parent configuration
        self.parent = parent
        self.path = path

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
