import ray
import argparse


def exit_handler(exit_handlers):
    print("Gracefully terminating")
    if exit_handlers is not None:
        for exit_handler in exit_handlers:
            if exit_handler is not None:
                exit_handler()
    ray.shutdown()
    exit(0)


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
                            # Run the expert
                            'expert',
                        ],
                        default='expert')

    args = parser.parse_args()

    def require_tasks():
        if args.tasks_path is None:
            print("Please supply tasks with --tasks_path")
            exit()

    if args.mode == 'expert':
        require_tasks()

    return args
