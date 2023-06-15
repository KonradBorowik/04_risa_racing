import cv2
import yaml
import numpy as np


class MapExtractor:
    def __init__(self) -> None:
        pass

    def convert_pgm_to_grid(self, pgm_file):
        # Load the .pgm file using OpenCV
        image = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
        # cv2.imwrite('original_map.jpg', image)
        
        # Threshold the image to separate obstacles and free space
        _, binary_image = cv2.threshold(image, 240, 255, cv2.THRESH_BINARY)
        
        # Invert the image so that obstacles are white (255) and free space is black (0)
        binary_image = cv2.bitwise_not(binary_image)
        
        # Convert the image to a numpy array
        grid = np.asarray(binary_image)

        rgb_image = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2RGB)
        
        y = rgb_image.shape[0]-258
        x = rgb_image.shape[1]-610
        print(y, x)
        cv2.circle(rgb_image, (x, y), 5, (255,0,0), -1)
        # cv2.imwrite('map.jpg', rgb_image) # for debugging purposes
        
        return grid

    def read_yaml(self, yaml_file):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)

        # Extract relevant parameters
        resolution = data['resolution']
        origin = data['origin']
        # Extract other necessary information as needed

        return resolution, origin