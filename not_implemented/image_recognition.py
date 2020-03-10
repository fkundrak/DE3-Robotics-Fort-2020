# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

from skimage.io import imread 
from skimage.feature import corner_fast, corner_peaks, corner_harris
from skimage.color import rgb2gray
import math
import numpy as np
import matplotlib.pyplot as plt


x_offset = -0.05
y_offset = -0.015

imc = imread('colourbrick.png')
im = rgb2gray(imc)

corner = corner_peaks(corner_harris(im), min_distance=5)

if (len(corner) == 4):
    
    #Fixed corner[0] as our starting point    (row first)
    
    line1len = math.sqrt(((corner[1,0]-corner[0,0])**2)+((corner[1,1]-corner[0,1])**2))
    line2len = math.sqrt(((corner[2,0]-corner[0,0])**2)+((corner[2,1]-corner[0,1])**2))
    line3len = math.sqrt(((corner[3,0]-corner[0,0])**2)+((corner[3,1]-corner[0,1])**2))
    linelen = np.sort([line1len, line2len, line3len])
    
    angle = np.arctan2(linelen[0], linelen[1])
    
    if (angle >= math.pi/2):
        angle = angle - math.pi

    if (angle <= -math.pi/2):
        angle = angle + math.pi

    centre_x = np.mean([corner[0,0],corner[1,0],corner[2,0],corner[3,0]])
    centre_y = np.mean([corner[0,1],corner[1,1],corner[2,1],corner[3,1]])
    
    scalefactor = 0.086/linelen[0]
    
    yerror = y_offset -(scalefactor * (centre_y- im.shape[1]/2)) #calculate x offset
    xerror = x_offset - (scalefactor * (centre_x- im.shape[0]/2)) #calculate y offset
   
#
    
else:
      #return xerror=100000, yerror=0, angle=0  # do something
      pass