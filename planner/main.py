from informed_rrt_star_3D import run_basic_env_informed_rrt
from rrt_star_3D import run_basic_env_rrt
import pickle
import datetime

# Setup parameters
motion_planing_algorithm = "informed_rrt_star"      # Choose between "rrt_star" and "informed_rrt_star"
obstacle_env = "simple"                           # Choose obstacle environment, choose between "standard", "simple", "complex" and "maze"
radius = 0.2                                        # Radius of the robot
draw_final_path = True                              # Draw the final path after the tree is generated
draw_every_node = True                              # Draw every node of the tree while generating it
iterations = 600                                      # Minimum number of iterations of rrt to perform
min_distance = 0.5                                    # Minimum distance between the goal and the final node of the tree

# Run the algorithm
if motion_planing_algorithm == "rrt_star":
    history = run_basic_env_rrt(obstacle_env, radius, draw_final_path, draw_every_node, iterations, min_distance, n_steps=10000, render=True, obstacles=False)
elif motion_planing_algorithm == "informed_rrt_star":
    history = run_basic_env_informed_rrt(obstacle_env, radius, draw_final_path, draw_every_node, iterations, min_distance, n_steps=10000, render=True, obstacles=False)

# Save history
now = datetime.datetime.now()
now.strftime("%H-%M-%S")

with open(f'final_paths/history-{obstacle_env}-run-{now}.pkl', 'wb') as f:
    pickle.dump(history, f)