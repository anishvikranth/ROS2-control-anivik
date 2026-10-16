import cv2
import numpy as np

print("1")

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
print("2")

p = cv2.aruco.DetectorParameters()
print("3")

img = np.zeros((10,10), np.uint8)
print("4")

print(type(d))
print(type(p))

print("5")

cv2.aruco.detectMarkers(img, d, parameters=p)

print("6")