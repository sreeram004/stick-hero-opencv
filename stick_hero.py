import cv2
import time
from ppadb.client import Client
import math
import numpy as np
from scipy.spatial import distance as dist

adb = Client(host='127.0.0.1', port=5037)
devices = adb.devices()

if len(devices) == 0:
    print('no device attached')
    quit()

device = devices[0]
  
def convert_to_cv2_img(image):
    
    with open('screen.png', 'wb') as f:
        f.write(image)
        
    img = cv2.imread("screen.png")
    
    f = int(img.shape[0] * 0.2)
    
    # crop image to remove 40% from top and 20% from bottom of the image
    img = img[f*2:-f, 0:]
    
    # resize image to a fixed size
    img = cv2.resize(img, (1080, 1960))
    
    return img

def get_centroid(c):
    
    M = cv2.moments(c)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    
    return [cX, cY]


def find_starting_pillar(d):
    return min(d.items(), key=lambda x:x[1]['centroid'][0])

def find_ending_pillar(d):
    return max(d.items(), key=lambda x:x[1]['centroid'][0])

def get_distance_to_move(contours):
    
    # sort and keep only countours with top 2 areas -- this will be our pillers
    contours_filtered = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)[:2]  
    
    # we now know the countours but we dont know which one is the start and which is end
    # to figure that out, compute centroid also, x,y,w,h of the bounding rectangle
    # add these info to a dict
    d = dict()
    for idx, c in enumerate(contours_filtered):
        
        x,y,w,h = cv2.boundingRect(c)
        centroid = get_centroid(c)
        
        d[idx] = {
            "edges" : [x, y, w, h],
            "centroid": centroid
        }

    # find the start and end pillars
    start = find_starting_pillar(d)[1]
    end = find_ending_pillar(d)[1]
    
    
    # lists for x,y cordinate of start and end
    start_xy = [-1,  -1]
    end_xy = end.get("centroid")
    
    # set x cordinate of start point as x of rectangle + width
    # set y cordinate of start point as end_xy's y cordinate
    x,y,w,h = start.get("edges")
    start_xy[0] = x + w
    start_xy[1] = end_xy[1]
            
    # find distance as euclidiean distance and round adjust it by 0.98 multiplication and floor
    distance = int(math.floor(dist.euclidean(start_xy, end_xy)) * 0.98)
    
    return distance
       

def play():
    
    # capture screenshot
    cap = device.screencap()
    
    # convert image to opencv format and then to HSV
    img = convert_to_cv2_img(cap)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # range for black -- color of pillars
    l_range_low = np.array([0, 0, 0])
    l_range_high = np.array([180, 255, 30])
    
    # threshold and convert image to binary
    img_bin = cv2.inRange(img_hsv, l_range_low, l_range_high)
    
    # find all countours
    contours, hierarchy = cv2.findContours(img_bin,  
                    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
    
    # helper function to get distance
    distance = get_distance_to_move(contours)
        
    # give a touch input for distance milli seconds at any point in the screen
    device.shell(f'input touchscreen swipe 500 500 500 500 {distance}')
    
    return distance


for jump in range(1, 1001, 1):
    print(f"Jump no : {jump}")
    distance = play()
    print(f"Distance : {distance}")
    time.sleep(3)
    