import cv2

# Load your marker image directly from package models path
image_path = "src/rover_description/models/marker_0/materials/textures/Marker_0.png"
img = cv2.imread(image_path)

# Test Dict 6x6_250
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

corners, ids, rejected = detector.detectMarkers(img)
print(f"Detected IDs with 6x6_250: {ids}")