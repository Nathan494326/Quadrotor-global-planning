import random
from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
from mpscenes.obstacles.urdf_obstacle import UrdfObstacle
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.cylinder_obstacle import CylinderObstacle
# Manually chosen positions for the first environment
cylinder_positions = [
    [7.0, 2.0, 5.0],
    [-4.0, -6.0, 5.0],
    [-5.0, 0.0, 5.0],
    [-1.0, 1.0, 5.0],
]

sphere_positions = [
    [-1.0, 7.0, 4.0],
]

cube_positions = [
    [3.0, -3.0, 5.0],
    [3.0, 3.0, 5.0],
    [-3.0, -3.0, 5.0],
]

# Create cylinders
cylinder_obstacles_manual = [
    CylinderObstacle(name=f"cylinder_manual_{i}", content_dict={
        "type": "cylinder",
        "movable": False,
        "geometry": {
            "position": pos,
            "radius": 1.0,
            "height": 10.0,
        },
        "rgba": [0.1, 0.3, 0.3, 1.0],
    }) for i, pos in enumerate(cylinder_positions)
]

# Create spheres
sphere_obstacles_manual = [
    SphereObstacle(name=f"sphere_manual_{i}", content_dict={
        "type": "sphere",
        "movable": False,
        "geometry": {"position": pos, "radius": 4.0},
    }) for i, pos in enumerate(sphere_positions)
]

# Create cubes
cube_obstacles_manual = [
    BoxObstacle(name=f"cube_manual_{i}", content_dict={
        "type": "box",
        "geometry": {
            "position": pos,
            "width": 1,
            "height": 10.0,
            "length": 1.5,
        },
    }) for i, pos in enumerate(cube_positions)
]

wall_length = 20
int_walls_length = 12
wall_obstacles_dicts = [
    {
        'type': 'box', 
         'geometry': {
             'position': [wall_length/2.0, 0.0, 1.5], 'width': wall_length, 'height': 3.0, 'length': 0.1
        },
        'high': {
            'position' : [wall_length/2.0, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [wall_length/2.0, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.1,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, wall_length/2.0, 1.5], 'width': 0.1, 'height': 3.0, 'length': wall_length
        },
        'high': {
            'position' : [0.0, wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length,
        },
        'low': {
            'position' : [0.0, wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, -wall_length/2.0, 1.5], 'width': 0.1, 'height': 3.0, 'length': wall_length
        },
        'high': {
            'position' : [0.0, -wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length,
        },
        'low': {
            'position' : [0.0, -wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [-wall_length/2.0, 0.0, 1.5], 'width': wall_length, 'height': 3.0, 'length': 0.1
        },
        'high': {
            'position' : [-wall_length/2.0, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [-wall_length/2.0, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.1,
        },
    },
]

wall_obstacles = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(wall_obstacles_dicts)]


# Combine all obstacles for the first environment
obstacles_env1 = wall_obstacles + cylinder_obstacles_manual + sphere_obstacles_manual + cube_obstacles_manual