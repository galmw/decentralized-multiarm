from .robot_env import RobotEnv, MultiRobotEnv
import itertools


class RobotUR5Env(RobotEnv):
    def __init__(self, ur5) -> None:
        self.ur5 = ur5

    def sample_config(self):
        return tuple(self.ur5.arm_sample_fn())

    def check_collision(self, q):
        # TODO move all other arms before using this thing so it actually works
        self.ur5.set_arm_joints(q)
        return self.ur5.check_collision()

    def distance(self, q1, q2):
        return self.ur5.arm_distance_fn(q1, q2)

    def extend(self, q1, q2):
        return self.ur5.arm_extend_fn(q1, q2)


class MultiRobotUR5Env(MultiRobotEnv):
    def __init__(self, ur5_group, resolutions) -> None:
        self.ur5_group = ur5_group
        self.robot_envs = [RobotUR5Env(ur5) for ur5 in ur5_group.active_controllers]
    
        self.check_multiple_collision_fn = self.ur5_group.get_collision_fn()
        self.extend_fn = self.ur5_group.get_extend_fn(resolutions)

    def sample_free_multi_config(self):
        """
        Returns array of randomly sampled node in composite config space.
        """
        return [robot_env.sample_free_config() for robot_env in self.robot_envs]

    def composite_distance(self, q1, q2):
        """
        Computes distance in "composite configuration space".
        Defined as sum of Euclidean distances between PRM nodes in two configs.
        """
        return self.ur5_group.distance_fn(q1, q2)

    def check_multiple_collision(self, q):
        return self.check_multiple_collision_fn(q)
    
    def extend(self, q1, q2):
        return self.extend_fn(list(itertools.chain.from_iterable(q1)), list(itertools.chain.from_iterable(q2)))