import argparse


def main_parse_args():
    parser = argparse.ArgumentParser("Centralized Multi Arm")
    parser.add_argument("--tasks-path", type=str, default=None,
                        help="path of directory containing tasks")
    parser.add_argument('--gui', action='store_true',
                        default=False, help='Run headless or render')
    parser.add_argument('--visualize', action='store_true',
                        default=False, help='If running with gui - show running process (slows down performance)')
    parser.add_argument('--mode',
                        choices=[
                            'rrt',
                            'mrdrrt'
                        ],
                        default='rrt')
    parser.add_argument('--cache-roadmaps', action='store_true',
                        default=True, help='Cache drrt roadmap graphs for later use')
    parser.add_argument('--num-prm-nodes', type=int,
                        default=50, help='How many nodes to setup each PRM roadmap')
    args = parser.parse_args()

    def require_tasks():
        if args.tasks_path is None:
            print("Please supply tasks path with --tasks_path")
            exit()
    
    if args.visualize and not args.gui:
        print("Cannot visualize run without showing GUI.")

    if args.mode == 'rrt' or args.mode == 'drrt':
        require_tasks()

    return args


import argparse


def task_designer_parse_args():
    parser = argparse.ArgumentParser("Scene designer")
    parser.add_argument("--tasks-path", type=str, default=None,
                        help="path of directory containing tasks")
    parser.add_argument('--gui', action='store_true',
                        default=False, help='Run headless or render')

    args = parser.parse_args()

    def require_tasks():
        if args.tasks_path is None:
            print("Please supply tasks path with --tasks_path")
            exit()
    
    require_tasks()

    return args
