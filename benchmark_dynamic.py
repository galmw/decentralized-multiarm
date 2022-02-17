# if __name__ == "__main__":
#     args = parse_args()
#     config = load_config(args.config)
#     env_conf = config['environment']
#     training_conf = config['training']
#     ray.init()
#     signal(SIGINT, lambda sig, frame: exit())
#     output_path = 'rrt_dynamic_benchmark_score.pkl'
#     benchmark_results = []
#     continue_benchmark = False
#     if exists(output_path):
#         # continue benchmark
#         benchmark_results = pickle.load(open(output_path, 'rb'))
#         continue_benchmark = True
#         finished_task_paths = [r['task']['task_path']
#                                for r in benchmark_results]

#     task_loader = TaskLoader(
#         root_dir=args.tasks_path,
#         shuffle=False,
#         repeat=False)
#     # Can change shuffle to True at some later stage
#     training_conf['task_loader'] = task_loader

#     RealTimeEnv = ray.remote(RealTimeEnv)
#     envs = [RealTimeEnv.remote(
#         env_config=env_conf,
#         training_config=training_conf,
#         gui=args.gui,
#         logger=None)
#         for _ in range(args.num_processes)]
#     env_pool = Pool(envs)

#     # env = RealTimeEnv(
#     #     env_config=env_conf,
#     #     training_config=training_conf,
#     #     gui=args.gui,
#     #     logger=None)

#     def callback(result):
#         benchmark_results.append(result)
#         if len(benchmark_results) % 100 == 0\
#                 and len(benchmark_results) > 0:

#             print('Saving benchmark scores to ',
#                   output_path)
#             with open(output_path, 'wb') as f:
#                 pickle.dump(benchmark_results, f)

#     def pbar_update(pbar):
#         pbar.set_description(
#             'Average Success Rate : {:.04f}'.format(
#                 mean([r['success_rate']
#                       for r in benchmark_results])))
#     tasks = [t for t in task_loader
#              if not continue_benchmark
#              or t.task_path not in finished_task_paths]
#     if args.load:
#         with tqdm(tasks, dynamic_ncols=True, smoothing=0.01) as pbar:
#             for task in pbar:
#                 callback(env.solve_task(task))
#                 pbar_update(pbar)
#     else:

#         # for i in range(100):        
#         #     env.solve_task(tasks[i])
        
#         benchmark_results = env_pool.map(
#             exec_fn=lambda env, task: env.solve_task.remote(task),
#             iterable=tasks,
#             pbar_update=pbar_update,
#             callback_fn=callback
#         )
