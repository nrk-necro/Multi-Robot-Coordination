import cv2
import numpy as np
import yaml

# Define grid size (size of a mobile robot)
grid_size = 32

  """
  Analyzes the image, divides it into grids, identifies objects and saves data.

  Args:
    image_path: Path to the overhead image.
    obstacle_size_threshold: Minimum size (in pixels) to consider something an obstacle.

  Returns:
    A list of dictionaries, where each dictionary represents an object with its type ('robot', 'target_object', 'obstacle')
    and grid coordinates ([x, y]). 
  """

def analyze_image(image_path, obstacle_size_threshold):

  # Read the image
  image = cv2.imread(image_path)

  # Get image height and width
  height, width = image.shape[:2]

  # Calculate number of grids in each dimension
  num_grids_x = int(width / grid_size)
  num_grids_y = int(height / grid_size)

  # List to store identified objects
  agents = []
  obstacles = []
  target_objects = []
  
  
  # Aruco dictionary and parameters
  aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4)
  aruco_params = cv2.aruco.DetectorParameters_create()

  # Loop through each grid
  for y in range(num_grids_y):
    for x in range(num_grids_x):
      # Get the grid coordinates
      grid_x = x * grid_size
      grid_y = y * grid_size

      # Extract the current grid area
      grid_image = image[grid_y:grid_y+grid_size, grid_x:grid_x+grid_size]

      # Convert to HSV color space for better color detection
      hsv_image = cv2.cvtColor(grid_image, cv2.COLOR_BGR2HSV)

      # Define color ranges for target object (green)
      green_lower = np.array([40, 50, 50])
      green_upper = np.array([80, 255, 255])

      # Create a mask for the target object
      mask = cv2.inRange(hsv_image, green_lower, green_upper)

      # Check if there are enough green pixels to consider it a target object
      num_green_pixels = cv2.countNonZero(mask)
      if num_green_pixels > (grid_size * grid_size) // 2:
        # Add target object to list
        target_objects.append([grid_x, grid_y])

      # Aruco marker detection for robots
      # Look for markers in the grid area
      corners, ids, rejected = cv2.aruco.detectMarkers(grid_image, aruco_dict, parameters=aruco_params)

      # If a marker is found
      if ids is not None:
        # Get the first marker (assuming only one robot per grid)
        marker_id = ids[0][0]
        
        agents.append({
              "start": [grid_x, grid_y],
              "goal": [grid_x, grid_y],  
              "name": marker_id
          })
        
      # Define color ranges for obstacle (red)
      red_lower = np.array([0, 100, 100])
      red_upper = np.array([10, 255, 255])  # Adjust upper bound for brighter reds

      # Create a mask
      mask = cv2.inRange(hsv_image, red_lower, red_upper)

      # Check if there are enough red pixels to consider it a target object
      num_red_pixels = cv2.countNonZero(mask)
      if num_red_pixels > (grid_size * grid_size) // 2:
        # Add target object to list
        obstacles.append([grid_x, grid_y])
		
  
  # Save object data to text file (using a dictionary instead)
  data = {
      "agents": agents,
      "map": {
          "dimensions": [num_grids_x, num_grids_y],
          "obstacles": obstacles,
          "target_objects": target_objects
      }
     }
  
  with open("object_data.yaml", "w") as f:
     	 yaml.dump(data, f)


