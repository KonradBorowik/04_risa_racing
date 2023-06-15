import getpass
import matplotlib.pyplot as plt

from .utils.parse_map import MapExtractor
from .utils.path_planner import PathPlanner


def main():
    user = getpass.getuser()
    map_path = f'/home/{user}/autoware/autoware_map/imola'
    
    extractor = MapExtractor()
    grid = extractor.convert_pgm_to_grid(map_path + '/imola.pgm')
    
    resolution, origin = extractor.read_yaml(map_path + '/imola.yaml')

    planner = PathPlanner(grid, resolution, origin)
    trajectory = planner.calculate_path()

    # print(trajectory) 

    # # plt.figure()
    # # plt.imshow(grid, cmap='gray')
    # # # Plotting the trajectory
    # x_values = [int(point[0]) for point in trajectory]
    # y_values = [int(point[1]) for point in trajectory]
    # print(x_values)
    # print(y_values)
    # plt.plot(x_values, y_values, 'b-', markersize=12)

    # plt.savefig('path.jpg')
