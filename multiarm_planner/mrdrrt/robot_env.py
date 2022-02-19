from abc import ABCMeta, abstractmethod
import itertools


class RobotEnv(metaclass=ABCMeta):
    @abstractmethod
    def sample_config():
        pass

    @abstractmethod
    def check_collision(self, q):
        pass

    def sample_free_config(self):
        """
        Generates random configuration in free configuration space.
        """
        while True:
            rand_q = self.sample_config()
            if (not self.check_collision(rand_q)):
                return tuple(rand_q)
    
    @abstractmethod
    def distance(self, q1, q2):
        pass

    @abstractmethod
    def difference(self, q1, q2):
        pass
    
    @abstractmethod
    def extend(self, q1, q2):
        pass

    @abstractmethod
    def forward_kinematics(self, q):
        pass

    def draw_line_between_configs(self, q1, q2, path=[]):
        full_path = [q1] + path + [q2]
        points = list(map(self.forward_kinematics, full_path))
        for prev, curr in zip(points, points[1:]):
            self.draw_line(prev, curr)

    @abstractmethod
    def draw_line(self, p1, p2):
        pass

    def check_path_collision_free(self, q1, q2):
        """
        Checks if path is collision free between two collision free configs. if it is, return it.
        """
        path = list(self.extend(q1, q2))[:-1]
        # Note: can be improved using some sort of bisect selector.
        if not any(self.check_collision(q) for q in path):
            return path
        return None


class MultiRobotEnv(metaclass=ABCMeta):
    @abstractmethod
    def two_robots_collision_on_paths(self, robot1, path1, robot2, path2):
        pass

    @abstractmethod
    def sample_free_multi_config():
        pass

    @abstractmethod
    def composite_distance(self, q1, q2):
        """
        Computes distance in "composite configuration space".
        Defined as sum of Euclidean distances between PRM nodes in two configs.
        """
        pass

    @abstractmethod
    def check_multiple_collision(q):
        pass

    @abstractmethod
    def extend(q1, q2):
        pass

    @abstractmethod
    def multi_forward_kinematics(self, q):
        pass

    def draw_line_between_multi_configs(self, q1, q2, path=[]):
        full_path = [list(itertools.chain.from_iterable(q1))] + path + [list(itertools.chain.from_iterable(q2))]
        points = list(map(self.multi_forward_kinematics, full_path))
        for prev, curr in zip(points, points[1:]):
            for i in range(len(prev)):
                self.robot_envs[i].draw_line(prev[i], curr[i])

    def check_path_collision_free(self, q1, q2):
        """
        Checks if path is collision free between two collision free configs. if it is, return it.
        """
        path = list(self.extend(q1, q2))[:-1]
        # Note: can be improved using some sort of bisect selector.
        if not any(self.check_multiple_collision(q) for q in path):
            return path
        return None
    
    @abstractmethod
    def setup_single_prm(i, start_configs, goal_configs, **kwargs):
        """
        Extra code if necessary before running a single PRM.
        """
        pass