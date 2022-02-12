from environment.rrt import pybullet_utils
from .robot_env import RobotEnv, MultiRobotEnv
import itertools


class RobotUR5Env(RobotEnv):
    def __init__(self, ur5) -> None:
        self.ur5 = ur5

    def sample_config(self):
        return tuple(self.ur5.arm_sample_fn())

    def check_collision(self, q):
        self.ur5.set_arm_joints(q)
        return self.ur5.check_collision()

    def distance(self, q1, q2):
        return self.ur5.arm_distance_fn(q1, q2)

    def difference(self, q1, q2):
        return self.ur5.arm_difference_fn(q1, q2)

    def extend(self, q1, q2):
        return self.ur5.arm_extend_fn(q1, q2)

    def forward_kinematics(self, q):
        return self.ur5.forward_kinematics(q)[0]

    def draw_line(self, p1, p2):
        pybullet_utils.draw_line(p1, p2, rgb_color=self.ur5.color, width=1)


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

    def setup_single_prm(self, i, start_configs, goal_configs, **kwargs):
        ur5_poses = kwargs['ur5_poses']
        self.ur5_group.setup([ur5_poses[i]], [start_configs[i]], specific_ur5s=[i])

