import cv2 as cv
from cv2 import aruco
import os

# Generate the 4x4 dictionary with 50 markers
dicto = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Marker resolution
marker_size = 400

# Path where this script is located
current_path = os.path.dirname(os.path.abspath(__file__))

# Parent directory (rover_aruco package)
package_path = os.path.dirname(current_path)

# Create the markers directory if it doesn't exist
markers_dir = os.path.join(package_path, "markers")
os.makedirs(markers_dir, exist_ok=True)

# Generate 50 markers
for i in range(50):

    marker_image = aruco.generateImageMarker(dicto, i, marker_size)
    saving_path = os.path.join(markers_dir, f"marker_{i}.png")
    # print(saving_path)
    cv.imwrite(saving_path, marker_image)