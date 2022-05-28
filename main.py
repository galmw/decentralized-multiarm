from signal import signal, SIGINT
from argument_parser import main_parse_args
from multiarm_planner.multiarm_environment import MultiarmEnvironment
from multiarm_planner.tasks import TaskLoader
import summary
import json
import os
import itertools


def main(args):
    signal(SIGINT, lambda sig, frame: exit())
    task_loader = TaskLoader(root_dir=args.tasks_path, shuffle=False, repeat=False)
    tasks = [t for t in task_loader]
    multiarm_env = MultiarmEnvironment(gui=args.gui, visualize=args.visualize)

    if args.benchmark:
        benchmark_conf = load_benchmark_conf(args.benchmark_path)
        planners = benchmark_conf['planners']
        task_summary_list = list(itertools.chain.from_iterable([run_planner(multiarm_env, planner, tasks) for planner in planners]))
        summary.create_benchmark_summary(args.benchmark_path, task_summary_list)
    
    else:
        planner = {"name": "default", "algorithm": args.planner}
        run_planner(multiarm_env, planner, tasks)


def load_benchmark_conf(becnmark_path):
    benchmark_conf = os.path.join(becnmark_path, 'run_config.json')
    with open(benchmark_conf) as f:
        conf = json.load(f)
    return conf


def run_planner(multiarm_env, planner, tasks):
    summary_list = []
    for task in tasks:
        task_summary = run_task(multiarm_env, planner, task)
        summary_list.append(task_summary)
    return summary_list


def run_task(multiarm_env, planner, task):
    if planner['name'] == 'default':
        if planner['algorithm'] == 'mrdrrt':
            planner['cache_roadmaps'] = True
            planner['num_prm_nodes'] = 300
            planner['goal_biasing'] = 0.2

    if planner['algorithm'] == 'mrdrrt':
        path, iterations, run_time = multiarm_env.mrdrrt_from_task(task,
                                                                  cache_roadmaps=planner['cache_roadmaps'],
                                                                  num_prm_nodes=planner['num_prm_nodes'],
                                                                  goal_biasing=planner['goal_biasing'])
    elif planner['algorithm'] == 'birrt':
        path, iterations, run_time = multiarm_env.birrt_from_task(task)
    elif planner['algorithm'] == 'rrt':
        path, iterations, run_time = multiarm_env.birrt_from_task(task, rrt_only=True)
    else:
        raise Exception(f"No matching algorithm found for planner {planner['algorithm']}")
    task_summary = summary.TaskSummary(planner['name'], task.id, path, iterations, run_time)                                                       
    return task_summary


if __name__ == "__main__":
    args = main_parse_args()
    main(args)

