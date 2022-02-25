import itertools
import os
import random
from venv import create
import IPython
import numpy as np
import pybullet as p
from argument_parser import task_designer_parse_args
from multiarm_planner.utils import create_circular_poses
from multiarm_planner.rrt import MultiarmEnvironment
from multiarm_planner.rrt.pybullet_utils import pairwise_collision
from multiarm_planner.tasks import Task


class TaskDesigner(object):
    def __init__(self, args):
        self.args = args
        self.multiarm_env = MultiarmEnvironment(gui=args.gui)
        self.task = None

    def load_task_from_file(self, task_name):
        print("[TaskDesigner] Loading Task from task file {0}".format(task_name))
        path = os.path.join(self.args.tasks_path, task_name)
        self.task = Task.from_file(path)
        self.setup_task()

    def save_task_to_file(self, task_name):
        path = os.path.join(self.args.tasks_path, task_name)
        self.task.task_path = path
        self.task.save()

    def setup_task(self, view_start_config=True):
        if not self.task:
            print("Load task from file first or create one.")
            return
        if view_start_config:
            self.multiarm_env.setup_run(self.task.base_poses, self.task.start_config, self.task.target_eff_poses, self.task.obstacles)
        else:
            self.multiarm_env.setup_run(self.task.base_poses, self.task.goal_config, self.task.target_eff_poses, self.task.obstacles)

    @property
    def active_ur5s(self):
        return self.multiarm_env.ur5_group.active_controllers
    
    def reset_robots(self, ur5s_count, set_collision_free=True):
        self.multiarm_env.ur5_group.enable_ur5s(count=ur5s_count)
        self._randomize_positions()
        for ur5 in self.active_ur5s:
            ur5.reset()
        if set_collision_free:
            self.randomize_collision_free_configs()

    def _sample_random_base_position(self, space_range=(-1, 1)):
        return np.array([random.uniform(*space_range), random.uniform(*space_range), 0])

    def _randomize_positions(self, min_pair_robot_dist=0.2, space_range=(-1, 1)):
        positions = None
        assert len(self.active_ur5s) > 0
        while True:
            positions = []
            for _ in range(len(self.active_ur5s)):
                positions.append(self._sample_random_base_position(space_range))
            distances = [np.linalg.norm(pos1 - pos2) for pos1, pos2 in itertools.product(positions, repeat=2)]
            if all([d == 0.0 or (d > min_pair_robot_dist) for d in distances]):
                for ur5, position in zip(self.active_ur5s, positions):
                    ur5.set_pose([position, p.getQuaternionFromEuler([0, 0, random.uniform(0, np.pi * 2)])])
                return

    def randomize_collision_free_configs(self):
        while True:
            for ur5 in self.active_ur5s:
                ur5.reset()
                target_pose = ur5.workspace.point_in_workspace()
                ur5.set_target_end_eff_pos(target_pose)

            if not any([ur5.check_collision() or ur5.violates_limits() for ur5 in self.active_ur5s]):
                # Collision free initial states within joint limits
                break

    def fast_randomize_collision_free_configs(self, order_forward=True):
        ur5s_order = self.active_ur5s if order_forward else self.active_ur5s[::-1]
        original_poses = [ur5.pose for ur5 in ur5s_order]
        for ur5, pose in zip(self.active_ur5s, create_circular_poses(3, len(self.active_ur5s))):
            ur5.set_pose(pose)

        while True:
            retry = False
            for ur5, original_pose in zip(ur5s_order, original_poses):
                stop = 0
                while True:
                    ur5.set_pose(original_pose)
                    ur5.set_arm_joints(ur5.arm_sample_fn())
                    # target_pose = ur5.workspace.point_in_workspace()
                    # ur5.set_target_end_eff_pos(target_pose)
                    if not (ur5.check_collision() or ur5.violates_limits()):
                        # Collision free initial states within joint limits
                        break
                    if stop == 999:
                        # If stuck, restart
                        retry = True
                        break
                    stop += 1
                if retry:
                    break
            if retry:
                continue
            else:
                break
        
    def create_task(self, fast_randomize=False):
        if fast_randomize:
            self.fast_randomize_collision_free_configs(True)
        else:
            self.randomize_collision_free_configs()
        start_config = [ur5.get_arm_joint_values() for ur5 in self.active_ur5s]
        if fast_randomize:
            self.fast_randomize_collision_free_configs(True)
        else:
            self.randomize_collision_free_configs()
        goal_config = [ur5.get_arm_joint_values() for ur5 in self.active_ur5s]

        self.task = Task(base_poses=[ur5.get_pose() for ur5 in self.active_ur5s],
                    goal_config=goal_config,
                    start_config=start_config,
                    target_eff_poses=[ur5.get_end_effector_pose() for ur5 in self.active_ur5s])
        self.setup_task()

    def _sample_random_obstacle(self, scale=None, position_range=(-1, 1)):
        scale = scale or random.uniform(0.1, 0.5)
        position = [random.uniform(*position_range), random.uniform(*position_range), random.uniform(scale / 2, 0.7)]
        obs = {'urdf_file': 'cube.urdf', 'position': position, 'scale': scale}
        return obs

    def create_random_cube_obstacle(self, scale=None, position_range=(-1, 1)):
        while True:
            obs = self._sample_random_obstacle(scale, position_range)
            self.task.obstacles.append(obs)
            # Check collision with end configs
            self.setup_task(False)
            if any(ur5.check_collision() or ur5.violates_limits() for ur5 in self.active_ur5s):
                self.task.obstacles.remove(obs)
                continue
            # Check collision with start configs
            self.setup_task()
            cube_id = self.multiarm_env.obstacles[-1].body_id
            if any(ur5.check_collision() or ur5.violates_limits() for ur5 in self.active_ur5s):
                self.task.obstacles.remove(obs)
                continue
            # Check collision with other cubes
            if any(pairwise_collision(cube_id, o.body_id) for o in self.multiarm_env.obstacles[:-1]):
                self.task.obstacles.remove(obs)
                continue
            break


if __name__ == "__main__":
    args = task_designer_parse_args()
    d = TaskDesigner(args)
    IPython.embed()