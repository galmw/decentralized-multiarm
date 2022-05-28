import argparse


def main_parse_args():
    parser = argparse.ArgumentParser("Centralized Multi Arm")
    parser.add_argument("--tasks-path", type=str, default=None,
                        help="path of directory containing tasks")
    parser.add_argument('--gui', action='store_true',
                        default=False, help='Run headless or render')
    parser.add_argument('--visualize', action='store_true',
                        default=False, help='If running with gui - show running process (slows down performance)')
    parser.add_argument('--planner',
                        choices=[
                            'rrt',
                            'birrt',
                            'mrdrrt'
                        ],
                        default='birrt')
    parser.add_argument('--cache-roadmaps', action='store_true',
                        default=True, help='Cache drrt roadmap graphs for later use')
    parser.add_argument('--num-prm-nodes', type=int,
                        default=50, help='How many nodes to setup each PRM roadmap')
    parser.add_argument('--benchmark', action='store_true',
                        default=False, help='Run in benchmark mode (no GUI, without pausing between runs)')
    parser.add_argument('--benchmark-path', type=str, default=None,
                        help='Path to get benchmark config and store results.')

    args = parser.parse_args()

    if args.visualize and not args.gui:
        print("Cannot visualize run without showing GUI.")
        exit()

    if args.benchmark and args.gui:
        print("Cannot run in benchmark mode with GUI.")
        exit()

    if args.tasks_path is None:
        print("Please supply tasks path with --tasks_path")
        exit()

    if args.benchmark and not args.benchmark_path:
        print('Please supply --benchmarh-path.')
        exit()

    return args


def task_designer_parse_args():
    parser = argparse.ArgumentParser("Scene designer")
    parser.add_argument("--tasks-path", type=str, default=None,
                        help="path of directory containing tasks")
    parser.add_argument('--gui', action='store_true',
                        default=False, help='Run headless or render')

    args = parser.parse_args()

    if args.tasks_path is None:
        print("Please supply tasks path with --tasks_path")
        exit()
    
    return args
