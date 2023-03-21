import numpy as np

def calculate_quaternion(current_position, desired_position):
    # Calculate the vector pointing from current position to desired position
    vector = np.array(desired_position) - np.array(current_position)
    vector = vector / np.linalg.norm(vector)

    # Calculate the quaternion angles
    theta = np.arccos(np.dot([0, 0, 1], vector))
    axis = np.cross([0, 0, 1], vector)
    axis = axis / np.linalg.norm(axis)

    w = np.cos(theta/2)
    x = axis[0] * np.sin(theta/2)
    y = axis[1] * np.sin(theta/2)
    z = axis[2] * np.sin(theta/2)

    return (w, x, y, z)


print(calculate_quaternion([0,0,0], [4,0,0]))