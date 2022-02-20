from signal import signal, SIGINT
from argument_parser import main_parse_args
from multiarm_planner.rrt import MultiarmEnvironment
from multiarm_planner.tasks import TaskLoader


def main(args):
    signal(SIGINT, lambda sig, frame: exit())
    task_loader = TaskLoader(root_dir=args.tasks_path, shuffle=False, repeat=False)
    tasks = [t for t in task_loader]
    mutiarm_env = MultiarmEnvironment(gui=args.gui, visualize=args.visualize)

    for i in range(len(tasks)):
        if args.mode == 'mrdrrt':
            mutiarm_env.mrdrrt_from_task(tasks[i], cache_roadmaps=args.cache_roadmaps, num_prm_nodes=args.num_prm_nodes)
        elif args.mode == 'rrt':
            mutiarm_env.birrt_from_task(tasks[i])


if __name__ == "__main__":
    args = main_parse_args()
    main(args)

