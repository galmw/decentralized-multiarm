import ray
from signal import signal, SIGINT
from argument_parser import main_parse_args
from multiarm_planner.rrt import MultiarmEnvironment
from multiarm_planner.tasks import TaskLoader


def main(args):
    ray.init()
    signal(SIGINT, lambda sig, frame: exit())

    task_loader = TaskLoader(root_dir=args.tasks_path, shuffle=False, repeat=False)
    tasks = [t for t in task_loader]
    mutiarm_env = MultiarmEnvironment(gui=args.gui, visualize=args.visualize)

    for i in range(len(tasks)):
        # ray.get(rrt.birrt_from_task.remote(tasks[i]))
        if args.mode == 'mrdrrt':
            mutiarm_env.mrdrrt_from_task(tasks[i], cache_drrt=args.cache_drrt)
        elif args.mode == 'rrt':
            mutiarm_env.birrt_from_task(tasks[i])


if __name__ == "__main__":
    args = main_parse_args()
    main(args)

