import ray
from environment.rrt import RRTWrapper
from utils import (
    parse_args,
    load_config,
)
from environment import TaskLoader
from signal import signal, SIGINT


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

    for i in range(len(tasks)):
        #ray.get(rrt.birrt_from_task.remote(tasks[i]))
        rrt.birrt_from_task(tasks[i])
        #rrt.mrdrrt_from_task(tasks[i])

