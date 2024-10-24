import cv2
import numpy as np


def find_and_crop_white_boundary(original_image_path, image_path):
  """
  Identifies the white boundary bounding box in an image and saves the cropped region.

  Args:
      image_path: Path to the overhead image.
      cropped_image_path: Path to save the cropped image.
  """
  # Read the image
  image = cv2.imread(original_image_path)

  # Convert to grayscale
  gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

  # Threshold for white color (adjust based on lighting)
  thresh = 240

  # Binarize the image
  binary_image = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)[1]

  # Find contours
  contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

  # Find the largest white contour (assuming the boundary is the largest)
  largest_contour = None
  largest_area = 0
  for contour in contours:
    area = cv2.contourArea(contour)
    if area > largest_area:
      largest_contour = contour
      largest_area = area

  # If no significant white contour is found, raise an error
  if largest_contour is None:
    raise ValueError("No significant white boundary found.")

  # Get bounding rectangle of the largest contour
  x, y, w, h = cv2.boundingRect(largest_contour)

  # Extract the cropped region
  cropped_image = image[y:y+h, x:x+w]

  # Save the cropped image
  cv2.imwrite(image_path, cropped_image)
