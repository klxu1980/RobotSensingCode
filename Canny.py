from __future__ import print_function
import cv2 as cv
import argparse
import numpy as np
max_lowThreshold = 100
window_name = 'Edge Map'
title_trackbar = 'Min Threshold:'
ratio = 3
kernel_size = 3


def CannyThreshold(val):
    low_threshold = val
    img_blur = cv.blur(src_gray, (3,3))
    detected_edges = cv.Canny(img_blur, low_threshold, low_threshold*ratio, kernel_size)
    #mask = detected_edges != 0
    #dst = src * (mask[:,:,None].astype(src.dtype))

    edge = ((detected_edges != 0) * 255).astype(np.uint8)
    cv.imshow(window_name, edge)


parser = argparse.ArgumentParser(description='Code for Canny Edge Detector tutorial.')
parser.add_argument('--input', help='Path to input image.', default='fruits.jpg')
args = parser.parse_args()
src = cv.imread('lena.jpg')
if src is None:
    print('Could not open or find the image: ', args.input)
    exit(0)
src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
cv.namedWindow(window_name)
cv.createTrackbar(title_trackbar, window_name , 0, max_lowThreshold, CannyThreshold)
CannyThreshold(0)
cv.waitKey()