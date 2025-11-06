import numpy as np

def is_collision_free(start, end, obstacles, radius_robot):
    for obstacle in obstacles:
        if intersects(start, end, obstacle, radius_robot):
            return False
    return True

def intersects(start, end, obstacle, radius_robot):
    """
    Check if the line segment intersects the obstacle
    Input: start [x, y, z], end [x, y, z], obstacle [dict]
    Return: True if intersects, False if not
    """

    xysz = np.linspace(start, end, 30)
    
    for point in xysz:
        if inside(point, obstacle, radius_robot):
            return True

    return False

def inside(point, obstacle, radius_robot):
    """
    Check the type of the obstacle and call the appropriate function
    Input: point [x, y, z], obstacle [dict]
    Return: True if inside, False if outside
    """
    if obstacle._config.type == "box":
        return inside_box(point, obstacle, radius_robot)
    elif obstacle._config.type == "sphere":
        return inside_sphere(point, obstacle, radius_robot)
    elif obstacle._config.type == "cylinder":
        return inside_cylinder(point, obstacle, radius_robot)
    # elif obstacle._config.type == "splineSphere":
    #     return inside_spline_sphere(point, obstacle)
    # elif obstacle._config.type == "urdf":
    #     return inside_urdf(point, obstacle)
    else:
        raise Exception("Obstacle type not recognized")
    
def inside_box(point, obstacle, radius_robot):
    """
    Check if the point is inside the box
    Input: point [x, y, z], obstacle [dict]
    Return: True if inside, False if outside
    """

    # Get the box's position, width, height, and length
    pos = obstacle._config.geometry.position
    width = obstacle._config.geometry.width
    height = obstacle._config.geometry.height
    length = obstacle._config.geometry.length

    # Check if the point is inside the box
    if pos[0] - length/2 - radius_robot <= point[0] <= pos[0] + length/2 + radius_robot:
        if pos[1] - width/2 - radius_robot <= point[1] <= pos[1] + width/2 + radius_robot:
            if pos[2] - height/2 - radius_robot <= point[2] <= pos[2] + height/2 + radius_robot:
                return True

    return False

def inside_sphere(point, obstacle, radius_robot):
    """
    Check if the point is inside the sphere
    Input: point [x, y, z], obstacle [dict]
    Return: True if inside, False if outside
    """

    # Get the sphere's position and radius
    pos = obstacle._config.geometry.position
    radius = obstacle._config.geometry.radius

    # Check if the point is inside the sphere
    if np.linalg.norm(point - pos) <= radius + radius_robot:
        return True

    return False

def inside_cylinder(point, obstacle, radius_robot):
    """
    Check if the point is inside the cylinder
    Input: point [x, y, z], obstacle [dict]
    Return: True if inside, False if outside
    """

    # Get the cylinder's position, radius, and height
    pos = obstacle._config.geometry.position
    radius = obstacle._config.geometry.radius
    height = obstacle._config.geometry.height

    # Check if the point is inside the cylinder
    if np.linalg.norm(point[0:2] - pos[0:2]) <= radius + radius_robot:
        if pos[2] - height/2 - radius_robot <= point[2] <= pos[2] + height/2 + radius_robot:
            return True

    return False