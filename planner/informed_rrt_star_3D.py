import gymnasium as gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np

from scene_examples.obstacle_env1 import obstacles_env1
from scene_examples.obstacle_env2 import obstacles_env2
from scene_examples.obstacle_env3 import obstacles_env3
from scene_examples.obstacle_env4 import obstacles_env4
from scene_examples.goal import *
from is_collision_free import *
from urdfenvs.urdf_common.urdf_env import UrdfEnv

# A node from the RRT* tree is defined by its configuration (x,y,z), its cost (the distance traveled to get there) and its parent (previous node)
class Node:
    def __init__(self, config, cost=float('inf')):
        self.config = config
        self.cost = cost
        self.parent = None

# That function returns the euclidean distance between two nodes
def distance(node1, node2):
    return np.linalg.norm(node1-node2)

# That function returns a list of nodes belonging to the tree that are located within max_range from new_node
def find_neighbours(new_node, tree, max_range):
    neighbours = []
    for node in tree:
        if distance(new_node, node.config) < max_range:
            neighbours.append(node)
    return neighbours

def compute_transformation_matrix(A, B):
    # Compute midpoint E
    E = (A + B) / 2.0
    
    # Compute unit vector u_x
    u_AB = (B - A) / np.linalg.norm(B - A)

    # Choose two orthogonal vectors u_y and u_z
    if u_AB[1] == 0:
        u_y_z = 0
    else: 
        u_y_z = u_AB[-1]/u_AB[1]

    u_y = np.array([0, 1, -u_y_z])  # You can choose another direction based on your requirements
    u_z = np.cross(u_AB, u_y)

    # Normalize u_z to ensure it is a unit vector
    if np.linalg.norm(u_z) != 0:
        u_z /= np.linalg.norm(u_z) 

    # Construct the transformation matrix
    T = np.eye(4)
    R = np.column_stack((u_AB, u_y, u_z))
    T[:3, :3] = R
    T[:3, -1] = E

    return T

def compute_random_point(path, T):
    nodes = np.zeros((len(path), 3))
    for i in range(len(path)):
        nodes[i, :] = path[i].config
    column_ones = np.ones((len(path), 1))
    nodes = np.hstack((nodes, column_ones))
    nodes = np.linalg.inv(T).dot(nodes.T).T
    x_max, y_max, z_max = np.max(np.abs(nodes[:,0]), axis=0), np.max(np.abs(nodes[:,1]),axis=0), np.max(np.abs(nodes[:,2]), axis=0)
    return x_max+3, y_max, z_max

# That function generates an RRT* tree from the start_pos to the goal, considering the obstacles int_wall_obstacles in the environment
def path_rrt_star(start_pos, goal, int_wall_obstacles, env, radius, draw_every_node, iterations, min_distance):

    iter = 0
    tree = [Node(start_pos, 0)]
    proximity_to_goal = float('inf')
    closest_to_goal = 0
    T = compute_transformation_matrix(start_pos, goal)
    change_boundary_box = 0
    path_found = 0
    just_found = 0
    branch_length = 3

    while iter < iterations or proximity_to_goal > min_distance:
        if iter % 50==0:
            print("Current iteration:", iter)
            print("The closest node to the goal is at: ", proximity_to_goal,"m\n")

        if change_boundary_box or just_found:
            print("Reduced sampling volume at iteration: ", iter, "\n")
            path = [tree[closest_to_goal]]   

            while path[-1].parent != None:
                path.append(path[-1].parent)

            # Rearrange the list to have the path in the right direction (start to goal)
            final_path = []
            for i in range(len(path)):
                final_path.append(path[-i-1])
            boundaries = compute_random_point(final_path, T)

            change_boundary_box = 0
            just_found = 0
            
        if path_found:
            target_point = np.array([np.random.uniform(-boundaries[0], boundaries[0]), np.random.uniform(-boundaries[1], boundaries[1]), np.random.uniform(-boundaries[2], boundaries[2]), 1.0])
            target_point = T.dot(target_point.T)[:3]
            target_point[0] = max(-9.5, min(target_point[0], 9.5))
            target_point[1] = max(-9.5, min(target_point[1], 9.5))
            target_point[2] = max(0.0, min(target_point[2], 9.5))
        else:
            target_point = np.array([np.random.uniform(-9.5, 9.5), np.random.uniform(-9.5, 9.5), np.random.uniform(0.0, 9.5)])
        
        # Let's find the closest node to define the exact position of the new node
        min_dist_node = None
        min_dist = float('inf')
        for candidate_point in tree:
            dist = distance(target_point, candidate_point.config)
            if dist < min_dist:
                min_dist = dist
                min_dist_node = candidate_point
        if distance(target_point, min_dist_node.config) > branch_length:
            new_node_config = min_dist_node.config + branch_length*(target_point - min_dist_node.config)/min_dist
        else:
            new_node_config = target_point
            
        # Only continue if the new node can be connected to its closest node without colliding with an obstacle, try a new random node otherwise
        if is_collision_free(new_node_config, min_dist_node.config, int_wall_obstacles, radius):
            # Check if that future new node will be the closest to the goal
            if distance(new_node_config, goal) < proximity_to_goal:
                if proximity_to_goal > 3.0 and distance(new_node_config, goal) < 3.0:
                    just_found = 1
                    path_found = 1
                    branch_length = 1.5
                
                proximity_to_goal = distance(new_node_config, goal)
                closest_to_goal = len(tree)

            # Let's connect the new node to the tree
            min_cost_node = None
            min_cost = float('inf')
            neighbours = find_neighbours(new_node_config, tree, branch_length*3/2 )

            # Let's find the node that will represent the lowest cost for the new node
            for node in neighbours:
                # Check whether the considered node can be connected to the new node without any collision
                if is_collision_free(new_node_config, node.config, int_wall_obstacles, radius):
                    cost = node.cost + distance(node.config, new_node_config)
                    if cost < min_cost:
                        min_cost = cost
                        min_cost_node = node

            # Add the new node to the tree and specify the computed cost as well as the parent of that new node
            tree.append(Node(new_node_config, min_cost))
            tree[-1].parent = min_cost_node

            # draw the node
            if draw_every_node:
                node_content_dicta = goal1Dict
                node_content_dicta["desired_position"] = tree[-1].config.tolist()
                node_to_draw = StaticSubGoal(name=f"tree_point", content_dict=node_content_dicta)
                env.add_goal(node_to_draw)
            
            # Let's reconfigure some connections if possible thanks to this new node
            cost_closest = tree[closest_to_goal].cost
            for node in tree:
                if node.cost > tree[-1].cost + distance(node.config, tree[-1].config) and distance(node.config, tree[-1].config) < branch_length*3/2 and is_collision_free(new_node_config, node.config, int_wall_obstacles, radius):
                        node.parent = tree[-1]
                        node.cost = tree[-1].cost + distance(node.config, tree[-1].config)
                if proximity_to_goal < 3:
                    if tree[closest_to_goal].cost < cost_closest:
                        cost_closest = tree[closest_to_goal].cost 
                        change_boundary_box = 1

        iter += 1

    # Now that the tree is generated, the path from the start to the goal must be extracted
    # For this matter we start from the goal, and go back to the start using the parent argument of each node
    path = [tree[closest_to_goal]]    
    while path[-1].parent != None:
        path.append(path[-1].parent)
    path.append(tree[0])

    # Rearrange the list to have the path in the right direction (start to goal)
    final_path = []
    for i in range(len(path)):
        final_path.append(path[-i-1])
    # Print final statistics
        
    if final_path[-1].cost != float('inf'):
        print("Final path has been found!")
        print("Total iterations: ", iter)
        print("Final path length: ", final_path[-1].cost)
    else:
        print("Final path could not be found, please increase the sampling ratio in the is_collision_free function")

    return final_path
     
       
def run_basic_env_informed_rrt(obstacle_env, radius, draw_final_path, draw_every_node, iterations, min_distance, n_steps=10000, render=False, obstacles=False):
    robots = [
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    ]
    env: UrdfEnv = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )
    action = np.array([0.0, 0.0, 0.0])
    pos0 = np.array([-8.5, -8.5, 1.0])
    vel0 = np.array([0.0, 0.0, 0.0])
    goal = np.array([8.5, 8.5, 8.5])
    ob = env.reset(pos=pos0, vel=vel0)
    print(f"Initial observation : {ob}")

    # add obstacles to environment, save internal wall obstacles for collision checking
    # CAREFUL HERE WITH THE CONDITIONS TO SELECT THE RIGHT OBSTACLES
    if obstacle_env == "simple":
        obstacles_manual = obstacles_env1
    elif obstacle_env == "complex":
        obstacles_manual = obstacles_env2
    elif obstacle_env == "standard":
        obstacles_manual = obstacles_env3
    elif obstacle_env == "maze":
        obstacles_manual = obstacles_env4

    int_wall_obstacles = []
    for wall in obstacles_manual:
        env.add_obstacle(wall)
        if (wall._config.type == "box") and (wall.width() == 20 or wall.length() == 20):
            pass # do not add external walls to internal wall obstacles
        else:
            int_wall_obstacles.append(wall)

    history = []
    
    env.reconfigure_camera(13.0, 180.0, -90.01, (0, 0, 0))
    ob, _, terminated, _, info  = env.step(action)
    path = path_rrt_star(pos0, goal, int_wall_obstacles, env, radius, draw_every_node, iterations, min_distance)

    # for node in path:
    #     # print node pos and paretn
    #     print(node.config, node.parent)

    # Draw final path
    if draw_final_path:
        # Draw it many times to make it more visible
        for i in range(15):
            for i, node in enumerate(path):
                # interpolate between current node and previous node and draw additional nodes
                if i > 0:
                    current_node = node.config
                    previous_node = path[i-1].config
                    length = np.linalg.norm(current_node - previous_node)
                    step = 0.1
                    interpolations = np.linspace(previous_node, current_node, int(length/step))
                    for j, interpolation in enumerate(interpolations):
                        node_content_dicta = goal1Dict
                        node_content_dicta["desired_position"] = interpolation.tolist()
                        node_to_draw = StaticSubGoal(name=f"goal{i+1}", content_dict=node_content_dicta)
                        env.add_goal(node_to_draw)
                else:
                    node_content_dicta = goal1Dict
                    node_content_dicta["desired_position"] = node.config.tolist()
                    node_to_draw = StaticSubGoal(name=f"goal{i+1}", content_dict=node_content_dicta)
                    env.add_goal(node_to_draw)

    input("Press Enter to close environment...")

    # # For now the motion of the robot has been left out, the focus was on the path planning
    # for node in path:
    #     current_position = ob['robot_0']['joint_state']['position']

    #     diff = np.linalg.norm(node.config - current_position) 

    #     while diff > 0.1:
    #         current_position = ob['robot_0']['joint_state']['position']
    #         err = node.config - current_position
    #         action = 5*err / np.linalg.norm(err)
    #         diff = np.linalg.norm(node.config - current_position)
    #         action = np.array([0.0, 0.0, 0.0])
    #         ob, _, terminated, _, info  = env.step(action)
    #         history.append(ob)

    env.close()
    return history