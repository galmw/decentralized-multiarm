import os
import tabulate
import pickle


def save_path_to_file(file_path, robots_path):
        with open(file_path, "wb") as f:
            pickle.dump(robots_path, f)


def create_benchmark_summary(benchmark_path, summaries):
    import time
    results_dir = os.path.join(benchmark_path, "results_{0}".format(time.strftime('%Y-%m-%d__%H-%M')))
    os.mkdir(results_dir)

    # To save the actual run paths:
    # [save_path_to_file(os.path.join(results_dir, f'{summary.task_id}.p'), summary.path) for summary in summaries]

    # Create statistics table file
    headers = ['Planner Name', 'Task ID', 'Result movement path length', '# Iterations', 'Runtime']
    summary_table_list = [summary.to_summary_row() for summary in summaries]
    # Sort by task name for readability
    summary_table_list.sort(key=lambda row: row[1])
    summary_table = tabulate.tabulate(summary_table_list, headers=headers)
    print('\n' + summary_table)
    with open(os.path.join(results_dir, "summary.txt"), 'w') as f:
        f.write(summary_table)


class TaskSummary(object):
    def __init__(self, planner_name, task_id, path, num_iterations, run_time) -> None:
        self.planner_name = planner_name
        self.task_id = task_id
        self.path = path
        self.num_itereations = num_iterations
        self.run_time = run_time
    
    def to_summary_row(self):
        # Create the relevant data for the summary
        path_length = len(self.path) if self.path is not None else 'INF'
        return [self.planner_name, self.task_id, path_length, self.num_itereations, self.run_time]