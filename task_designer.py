import itertools
import os
import random
import IPython
import numpy as np
import pybullet as p
from argument_parser import task_designer_parse_args
from multiarm_planner.obstacles import Obstacle
from multiarm_planner.rrt import MultiarmEnvironment
from multiarm_planner.rrt.pybullet_utils import pairwise_collision
from multiarm_planner.tasks import Task


class TaskDesigner(object):
    def __init__(self, args):
        self.args = args
        self.multiarm_env = MultiarmEnvironment(gui=args.gui)
        self.task = None

    def load_task_from_file(self, task_name, view_start_config=True):
        print("[TaskDesigner] Loading Task from task file {0}".format(task_name))
        path = os.path.join(self.args.tasks_path, task_name)
        self.task = Task.from_file(path)
        self.setup_task(view_start_config)

    def save_task_to_file(self, task_name):
        path = os.path.join(self.args.tasks_path, task_name)
        self.task.task_path = path
        self.task.save()

    def setup_task(self, view_start_config=True):
        if view_start_config:
            self.multiarm_env.setup_run(self.task.base_poses, self.task.start_config, self.task.target_eff_poses, self.task.obstacles)
        else:
            self.multiarm_env.setup_run(self.task.base_poses, self.task.goal_config, self.task.target_eff_poses, self.task.obstacles)

    @property
    def active_ur5s(self):
        return self.multiarm_env.ur5_group.active_controllers
    
    def reset(self, ur5s_count):
        self.multiarm_env.ur5_group.enable_ur5s(count=ur5s_count)
        self._randomize_positions()
        for ur5 in self.active_ur5s:
            ur5.reset()
        self.randomize_collision_free_configs()

    def _sample_random_base_position(self):
        return np.array([random.uniform(-1, 1), random.uniform(-1, 1), 0])

    def _randomize_positions(self, min_pair_robot_dist=0.2):
        positions = None
        assert len(self.active_ur5s) > 0
        while True:
            positions = []
            for _ in range(len(self.active_ur5s)):
                if len(positions) == 0:
                    positions.append(self._sample_random_base_position())
                else:
                    positions.append(positions[-1] + self._sample_random_base_position())

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

            for _ in range(50):
                p.stepSimulation()
                for ur5 in self.active_ur5s:
                    ur5.step()

            if not any([ur5.check_collision() or ur5.violates_limits() for ur5 in self.active_ur5s]):
                # Collision free initial states within joint limits
                break

    def create_task_with_robots(self, ur5s_count):
        self.reset(ur5s_count)
        start_config = [ur5.get_arm_joint_values() for ur5 in self.active_ur5s]
        self.randomize_collision_free_configs()
        goal_config = [ur5.get_arm_joint_values() for ur5 in self.active_ur5s]

        self.task = Task(base_poses=[ur5.get_pose() for ur5 in self.active_ur5s],
                    goal_config=goal_config,
                    start_config=start_config,
                    target_eff_poses=[ur5.get_end_effector_pose() for ur5 in self.active_ur5s])

    def _sample_random_obstacle(self):
        scale = random.uniform(0.1, 0.5)
        position = [random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(scale / 2, 1)]
        obs = {'urdf_file': 'cube.urdf', 'position': position, 'scale': scale}
        return obs

    def create_random_cube_obstacle(self):
        while True:
            obs = self._sample_random_obstacle()
            # Check collision for both start and end configs
            self.task.obstacles.append(obs)
            self.setup_task(True)
            cube_id = self.multiarm_env.obstacles[-1].body_id
            if any(ur5.check_collision() or ur5.violates_limits() for ur5 in self.active_ur5s):
                self.task.obstacles.remove(obs)
                continue
            self.setup_task(False)
            if any(ur5.check_collision() or ur5.violates_limits() for ur5 in self.active_ur5s):
                self.task.obstacles.remove(obs)
                continue
            if any(pairwise_collision(cube_id, o.body_id) for o in self.multiarm_env.obstacles[:-1]):
                self.task.obstacles.remove(obs)
                continue
            break


if __name__ == "__main__":
    args = task_designer_parse_args()
    d = TaskDesigner(args)
    IPython.embed()