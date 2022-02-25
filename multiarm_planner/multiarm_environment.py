from itertools import chain
import pybullet as p
from time import sleep
from .obstacles import Obstacle
from .profile_utils import timefunc
from .ur5_group import UR5Group

from .rrt.rrt_connect import birrt
from .rrt.pybullet_utils import configure_pybullet, draw_line, remove_all_markers

from .robot_ur5_env import MultiRobotUR5Env
from .mrdrrt.mrdrrt_planner import MRdRRTPlanner


class MultiarmEnvironment:
    _MAX_UR5_COUNT = 10

    def __init__(self, gui=True, visualize=False):
        from multiarm_planner.utils import create_ur5s, Target
        print("[MultiarmEnv] Setting up multiarm environment")
        # set up simulator
        configure_pybullet(rendering=gui, debug=False, yaw=0, pitch=0, dist=1.0, target=(0, 0, 0.3))
        p.loadURDF("plane.urdf", [0, 0, -0.01])

        self.gui = gui
        self.visualize = visualize
        self.obstacles = None

        def create_ur5s_fn():
            return create_ur5s(radius=0.8, count=self._MAX_UR5_COUNT, speed=0.02)

        self.ur5_group = UR5Group(create_ur5s_fn=create_ur5s_fn, collision_distance=0)
        self.targets = [Target(pose=[[0, 0, 0], [0, 0, 0, 1]], color=ur5.color)
                        for ur5 in self.ur5_group.all_controllers]

    def setup_run(self, ur5_poses, start_conf, target_eff_poses, obstacles):
        if self.gui:
            remove_all_markers()
            for pose, target in zip(target_eff_poses, self.targets):
                target.set_pose(pose)

        self.ur5_group.setup(ur5_poses, start_conf)
        del self.obstacles
        self.obstacles = Obstacle.load_obstacles(obstacles)

    def birrt_from_task(self, task):
        print("[MultiarmEnv] Running BiRRT for task {0}".format(task.id))
        return self.birrt(start_configs=task.start_config,
                          goal_configs=task.goal_config,
                          ur5_poses=task.base_poses,
                          target_eff_poses=task.target_eff_poses,
                          obstacles=task.obstacles)

    def birrt(self, start_configs, goal_configs,
              ur5_poses, target_eff_poses, obstacles=None, resolutions=0.1, timeout=100000):
        self.setup_run(ur5_poses, start_configs, target_eff_poses, obstacles)

        extend_fn = self.ur5_group.get_extend_fn(resolutions)
        collision_fn = self.ur5_group.get_collision_fn()
        start_conf = list(chain.from_iterable(start_configs))
        goal_conf = list(chain.from_iterable(goal_configs))
        
        path = birrt(start_conf=start_conf,
                     goal_conf=goal_conf,
                     distance=self.ur5_group.distance_fn,
                     sample=self.ur5_group.sample_fn,
                     extend=extend_fn,
                     collision=collision_fn,
                     iterations=10000,
                     smooth=5,
                     visualize=self.visualize,
                     fk=self.ur5_group.forward_kinematics,
                     group=True,
                     greedy=True,
                     timeout=timeout)

        if path is None:
            return None
        if self.gui:
            self.demo_path(ur5_poses, start_configs, path)
        return path

    def mrdrrt_from_task(self, task, cache_roadmaps=True, num_prm_nodes=50):
        print("[MultiarmEnv] Running MrDRRT for task {0}".format(task.id))
        return self.mrdrrt(start_configs=task.start_config,
                           goal_configs=task.goal_config,
                           ur5_poses=task.base_poses,
                           target_eff_poses=task.target_eff_poses,
                           obstacles=task.obstacles,
                           task_path=task.task_path,
                           cache_roadmaps=cache_roadmaps,
                           num_prm_nodes=num_prm_nodes)
                           
    def mrdrrt(self, start_configs, goal_configs,
              ur5_poses, target_eff_poses, obstacles=None,
              resolutions=0.1, task_path=None, cache_roadmaps=True, num_prm_nodes=50):
        start_configs = tuple(tuple(conf) for conf in start_configs)
        goal_configs = tuple(tuple(conf) for conf in goal_configs)
        self.setup_run(ur5_poses, start_configs, target_eff_poses, obstacles)
        env = MultiRobotUR5Env(self.ur5_group, resolutions, self.obstacles)
        mrdrrt = MRdRRTPlanner(env, visualize=self.visualize)

        mrdrrt.get_implicit_graph(start_configs=start_configs, goal_configs=goal_configs, ur5_poses=ur5_poses,
                                    cache_roadmaps=cache_roadmaps, task_path=task_path, n_nodes=num_prm_nodes)

        self.ur5_group.setup(ur5_poses, start_configs)
        path = mrdrrt.find_path(start_configs, goal_configs)
        if path is None:
            return None

        if self.gui:
            self.demo_path(ur5_poses, start_configs, path)
        return path

    def demo_path(self, ur5_poses, start_configs, path_conf):
        self.ur5_group.setup(ur5_poses, start_configs)
        input("Press enter to play demo!")
        edges = []
        for i in range(len(path_conf)):
            if i != len(path_conf) - 1:
                for pose1, pose2 in zip(self.ur5_group.forward_kinematics(path_conf[i]),
                                        self.ur5_group.forward_kinematics(path_conf[i + 1])):
                    draw_line(pose1[0], pose2[0], rgb_color=[1, 0, 0], width=6)
                    edges.append((pose1[0], pose2[0]))
        for i, q in enumerate(path_conf):
            self.ur5_group.set_joint_positions(q)
            sleep(0.05)
        input("Done playing demo")


