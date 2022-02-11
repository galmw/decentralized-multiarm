import os
from environment.rrt import RRTWrapper
from utils import parse_args, load_config
from environment.tasks import Task


def work_with_scene(task_path):
    view_start = True
    while True:
        task = Task.from_file(task_path)
        rrt.load_scene_from_task(task, view_start=view_start)
        user_input = input("Enter nothing to reload and switch start and goal configs; Enter x to return to scene loading.\n")
        if user_input == "x":
            break
        view_start = not view_start


if __name__ == "__main__":
    args = parse_args()
    config = load_config(args.config)
    env_config = config['environment']
    rrt = RRTWrapper(env_config, gui=True)

    while True:
        user_input = input("Enter scene name to load scene; Enter 'x' to quit\n")
        if user_input == "x":
            break
        elif user_input != "":
            try:
                work_with_scene(os.path.join(args.tasks_path, user_input))
            except:
                print("Failed to load task.")

