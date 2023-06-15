import matplotlib.pyplot as plt

def visualize_trajectory(path, road_network):
    # Extract the coordinates of the nodes in the path
    path_coordinates = [road_network.nodes[node]['coordinates'] for node in path]

    # Extract the x and y coordinates separately
    x_coordinates = [coord[0] for coord in path_coordinates]
    y_coordinates = [coord[1] for coord in path_coordinates]

    # Plot the trajectory
    plt.figure(figsize=(8, 6))
    plt.plot(x_coordinates, y_coordinates, 'b-', linewidth=2)
    plt.scatter(x_coordinates, y_coordinates, color='r', marker='o')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Generated Trajectory')
    plt.grid(True)
    plt.savefig('path.pdf')
