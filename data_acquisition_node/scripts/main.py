"""
   main.py
   Script to read images and process them.   
   Matheus Nascimento
   April, 2018
   Based on Python 2.7 Version    
"""
import os
import cv2
import numpy as np
import sys
import re

if __name__ == '__main__':
    try:
        images_path = sys.argv[1]
    except IndexError:
        images_path = "/home/matheus/32bits/"
    
    dirFiles = os.listdir(images_path)
    ordered_files = sorted(dirFiles, key=lambda x: (int(re.sub('\D','',x)),x))

    for j in range (0,len(ordered_files) ):
        print images_path+ordered_files[j]
        mat = cv2.imread(images_path+ordered_files[j], -1)
        mat = np.float32(mat)
        print mat.shape
        print mat.dtype
        cv2.namedWindow('PNG Frames', cv2.WINDOW_NORMAL)
        cv2.imshow('PNG Frames',mat)
        cv2.waitKey(200)

    cv2.destroyAllWindows()