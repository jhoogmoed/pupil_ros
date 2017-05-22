#!/usr/bin/env python
import numpy as np
from PIL import Image, ImageDraw
from collections import Counter

#Define colors
yellow = (255, 255, 0)
orange = (255, 138, 0)
red = (255, 0, 0)
green = (0, 255, 0)
purple = (255, 0, 255)
blue = (0, 0, 255)
lightblue = (0, 255, 240)
white = (255, 255, 255)
c_det = ''
class detection:
    @staticmethod
    def color(img,x,y):
        # Open image
        im = Image.fromarray(img, 'RGB')
        width, height = im.size

        #Convert to rgb
        rgb_im = im.convert('RGB')

        #Set gaze radius for most frequent color
        radius = 20

        #Get all pixels around x and y
        array = []
        for i in range (-radius, radius):
            for j in range (-radius,radius):
                if (i**2 + j**2) <= radius**2:
                    b, g, r = rgb_im.getpixel((x+i,y+j))
                    array.append((r,g,b))

        #Get most frequent value
        rank = 0
        count = Counter(array)
        color = count.most_common()[rank]
        aantal = color[1]
        color =  color[0]
        conf = float(aantal) / np.shape(array)[0]
        rank += 1

        #Set color or object to rgb values
        if np.allclose(color,orange):
            c_det = "oranje"
        elif np.allclose(color,yellow):
            c_det = "geel"
        elif np.allclose(color,red):
            c_det = "Person"
        elif np.allclose(color,green):
            c_det = "groen"
        elif np.allclose(color,purple):
            c_det = "paars"
        elif np.allclose(color,blue):
            c_det = "Car"
        elif np.allclose(color,lightblue):
            c_det = "licht blauw"
        elif np.allclose(color,white):
            c_det = "wit"
        else:
            c_det = "Unknown"

        return r,g,b, c_det, conf
