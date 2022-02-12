from abc import ABCMeta, abstractmethod

from environment.rrt.pybullet_utils import forward_kinematics


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

    def draw_line_between_configs(self, q1, q2):
        p1 = self.forward_kinematics(q1)
        p2 = self.forward_kinematics(q2)
        self.draw_line(p1, p2)

    @abstractmethod
    def draw_line(self, p1, p2):
        pass

    def is_edge_collision_free(self, q1, q2):
        return not any(self.check_collision(q) for q in self.extend(q1, q2))


class MultiRobotEnv(metaclass=ABCMeta):
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

    def draw_line_between_multi_configs(self, q1, q2):
        for i, (q1_robot, q2_robot) in enumerate(zip(q1, q2)):
            self.robot_envs[i].draw_line_between_configs(q1_robot, q2_robot)

    def is_edge_collision_free(self, q1, q2):
        return not any(self.check_multiple_collision(q) for q in self.extend(q1, q2))

    @abstractmethod
    def setup_single_prm(i, start_configs, goal_configs, **kwargs):
        """
        Extra code if necessary before running a single PRM.
        """
        pass
