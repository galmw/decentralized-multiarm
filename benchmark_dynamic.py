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
