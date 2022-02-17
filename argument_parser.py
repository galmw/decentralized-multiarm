import argparse


def parse_args():
    parser = argparse.ArgumentParser("Centralized Multi Arm")
    parser.add_argument("--tasks_path", type=str, default=None,
                        help="path of directory containing tasks")
    parser.add_argument('--gui', action='store_true',
                        default=False, help='Run headless or render')
    parser.add_argument('--num_processes', type=int,
                        default=16, help='How many processes to parallelize')
    parser.add_argument('--mode',
                        choices=[
                            'rrt',
                            'mrdrrt'
                        ],
                        default='rrt')
    parser.add_argument('--cache-drrt', action='store_true',
                        default=True, help='Cache drrt implicit graphs for later use')

    args = parser.parse_args()

    def require_tasks():
        if args.tasks_path is None:
            print("Please supply tasks with --tasks_path")
            exit()

    if args.mode == 'rrt' or args.mode == 'drrt':
        require_tasks()

    return args
