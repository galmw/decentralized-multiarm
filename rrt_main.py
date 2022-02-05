import ray
from environment.rrt import RRTWrapper
from environment import utils
from environment import RealTimeEnv
from utils import (
    parse_args,
    load_config,
    create_policies,
    exit_handler
)
from environment import TaskLoader
import pickle
from signal import signal, SIGINT
from numpy import mean
from distribute import Pool
from os.path import exists
from tqdm import tqdm
import time


if __name__ == "__main__":
    args = parse_args()
    config = load_config(args.config)
    env_config = config['environment']

    ray.init()
    signal(SIGINT, lambda sig, frame: exit())
    output_path = 'rrt_dynamic_benchmark_score.pkl'
    benchmark_results = []

    task_loader = TaskLoader(
        root_dir=args.tasks_path,
        shuffle=False,
        repeat=False)

    tasks = [t for t in task_loader]
 
    #rrt = RRTWrapper.remote(env_config, gui=True)
    rrt = RRTWrapper(env_config, gui=True)

    for i in range(10):
        #ray.get(rrt.birrt_from_task.remote(tasks[i]))
        #rrt.birrt_from_task(tasks[i])
        rrt.mrdrrt_from_task(tasks[i])


    # benchmark_results = env_pool.map(
    #     exec_fn=lambda env, task: env.solve_task.remote(task),
    #     iterable=tasks,
    #     pbar_update=pbar_update,
    #     callback_fn=callback
    # )
