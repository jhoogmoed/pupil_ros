from PIL import Image, ImageDraw
import numpy as np
from collections import Counter

# Defineren van kleuren
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

        # x_norm = 0.4
        # y_norm = 0.6

        # x = x_norm*width
        # y = (1 - y_norm)*height

        rgb_im = im.convert('RGB')

        radius = 20
        array = []
        # if x < width and y < height:
        # Alle pixels ophalen rond x,y coordinaten
        for i in range (-radius, radius):
            for j in range (-radius,radius):
                if (i**2 + j**2) <= radius**2:
                    b, g, r = rgb_im.getpixel((x+i,y+j))
                    array.append((r,g,b))

        # Meest aanwezige waarde uit array halen
        rank = 0
    
        count = Counter(array)
        color = count.most_common()[rank]
        aantal = color[1]
        color =  color[0]

        conf = float(aantal) / np.shape(array)[0]
        rank += 1

        # if (rank > 1) and (conf < 0.01):
        #     break

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
            c_det = "geen idee welke kleur dit is, flikker op"
        # print "percentage:", conf

        return r,g,b, c_det, conf

        # draw = ImageDraw.Draw(im)
        # draw.ellipse((x-radius,y-radius,x+radius,y+radius),fill=None,outline=(0,0,0))
        # draw.ellipse((x-2,y-2,x+2,y+2),fill=(0,0,0),outline=(0,0,0))
        # im.show()