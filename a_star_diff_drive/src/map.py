import cv2
import numpy as np

def map(clearance, radius):
    block = clearance + round(radius)
    map_canvas = np.zeros((200, 600, 3))
    map_canvas_bw = np.zeros((200, 600))

    cv2.rectangle(map_canvas, (0, 0), (600, 200), (255, 255, 255), block)
    cv2.rectangle(map_canvas, (150, 75), (165, 200), (255, 255, 255), block)
    cv2.rectangle(map_canvas, (250, 0), (265, 125), (255, 255, 255), block)
    cv2.circle(map_canvas, (400, 110), 50, (255, 255, 255), block)
    cv2.rectangle(map_canvas, (150, 75), (165, 200), (150, 100, 100), -1)
    cv2.rectangle(map_canvas, (250, 0), (265, 125), (150, 100, 100), -1)
    cv2.circle(map_canvas, (400, 110), 50, (150, 100, 100), -1)
    

    cv2.rectangle(map_canvas_bw, (0, 0), (600, 200), (255, 255, 255), block)
    cv2.rectangle(map_canvas_bw, (150, 75), (165, 200), (255, 255, 255), block)
    cv2.rectangle(map_canvas_bw, (250, 0), (265, 125), (255, 255, 255), block)
    cv2.circle(map_canvas_bw, (400, 110), 50, (255, 255, 255), block)
    cv2.rectangle(map_canvas_bw, (150, 75), (165, 200), (150, 100, 100), -1)
    cv2.rectangle(map_canvas_bw, (250, 0), (265, 125), (150, 100, 100), -1)
    cv2.circle(map_canvas_bw, (400, 110), 50, (150, 100, 100), -1)

    return map_canvas, map_canvas_bw

def free_points(map):
    y,x = np.where(map==0)
    free_points = []
    for i,j in zip(x,y):
        free_points.append((i,j))
    return free_points

