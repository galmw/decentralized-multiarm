import os
from argument_parser import parse_args
from multiarm_planner.rrt import MultiarmEnvironment
from multiarm_planner.tasks import Task


def work_with_scene(multiarm_env, task_path):
    view_start = True
    while True:
        task = Task.from_file(task_path)
        multiarm_env.load_scene_from_task(task, view_start=view_start)
        user_input = input("Enter nothing to reload and switch start and goal configs; Enter x to return to scene loading.\n")
        if user_input == "x":
            break
        view_start = not view_start


def scene_desginer(args):
    multiarm_env = MultiarmEnvironment(gui=True)
    while True:
        user_input = input("Enter scene name to load scene; Enter 'x' to quit\n")
        if user_input == "x":
            break
        elif user_input != "":
            try:
                work_with_scene(multiarm_env, os.path.join(args.tasks_path, user_input))
            except:
                print("Failed to load task.")


if __name__ == "__main__":
    args = parse_args()
    scene_desginer(args)
    