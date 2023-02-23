# -------------------
#
# Welcome to LMAO - Localization Mapping and Optimization (or Odometry)
# This is a library that will include the classes and functions needed to perform localization, mapping and optimization in a 3D environment.
# The library is built on top of Open3D, and uses raycasting in a triangle mesh.
# It is made by Rasmus Peter Junge, and Daniel Gahner Holm for our Master's Thesis at SDU.
#
# -------------------

# Path: lmao/mapping.py

import cv2

def find_edges(img):
    # Find edges in image
    edges = cv2.Canny(img, 100, 200)
    return edges

def get_derivative(img, ksize=5):
    # Get derivative of image
    dx = cv2.Sobel(img, cv2.CV_16S, 1, 0, ksize=ksize*2+1)
    dy = cv2.Sobel(img, cv2.CV_16S, 0, 1, ksize=ksize)
    abs_dx = cv2.convertScaleAbs(dx)
    abs_dy = cv2.convertScaleAbs(dy)
    grad = cv2.addWeighted(abs_dx, 0.5, abs_dy, 0.5, 0)
    return grad
