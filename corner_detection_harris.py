# sry, too tired to write comments, if you get the harris data out of it you'll get the relative coordinates of the dots

import cv2 as cv
import numpy as np


video = cv.VideoCapture('video.mp4')
while True:

    ret, frame = video.read()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gray2 = np.float32(gray)

    harris = cv.cornerHarris(gray2, 5, 3, 0.243)
    
    print("\n\n")
    print(harris)
    cv.imshow('harris', harris)
    #cv.imshow('frame', frame)
    #cv.imshow('gray', gray)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv.destroyAllWindows()
