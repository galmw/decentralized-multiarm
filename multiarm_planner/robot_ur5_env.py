import itertools
from .rrt import pybullet_utils
from .mrdrrt.robot_env import RobotEnv, MultiRobotEnv


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
    
        # TODO so theoretically this function shouldn't even exist or be used
        self.check_multiple_collision_fn = self.ur5_group.get_collision_fn()
        # Same for this one
        self.extend_fn = self.ur5_group.get_extend_fn(resolutions)

    def two_robots_collision_on_paths(self, robot1, path1, robot2, path2):
        ur5_a = self.ur5_group.active_controllers[robot1]
        ur5_b = self.ur5_group.active_controllers[robot2]
        for q1, q2 in zip(path1, path2):
            ur5_a.set_arm_joints(q1)
            ur5_b.set_arm_joints(q2)
            if pybullet_utils.pairwise_collision(ur5_a.body_id, ur5_b.body_id):
                return True
        if len(path1) > len(path2):
            for q1 in path1[len(path2):]:
                ur5_a.set_arm_joints(q1)
                if pybullet_utils.pairwise_collision(ur5_a.body_id, ur5_b.body_id):
                    return True
        elif len(path2) > len(path1):
            for q2 in path2[len(path1):]:
                ur5_b.set_arm_joints(q2)
                if pybullet_utils.pairwise_collision(ur5_a.body_id, ur5_b.body_id):
                    return True
        return False

    def multi_forward_kinematics(self, q):
        return [self.robot_envs[i].forward_kinematics(q[i*6:(i + 1)*6]) for i in range(len(q) // 6)]

    def sample_free_multi_config(self):
        """
        Returns array of randomly sampled node in composite config space.
        """
        return tuple(robot_env.sample_free_config() for robot_env in self.robot_envs)

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
