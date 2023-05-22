import cv2
import numpy as np
import time

def viz(img, parent_node, visited, path, free_points):
    value = int(input("for viewing the video press 1 "))
    total = visited + path
    visited_len = len(visited)
    i = 0
    for index in range(len(total)-1):
        if i<visited_len:
            img[int(total[index][1]), int(total[index][0])] = (0,255,0)
        else:
            point1 = (int(total[index][0][0]), int(total[index][0][1]))
            point2 = (int(total[index+1][0][0]), int(total[index+1][0][1]))
            img = cv2.line(img, point1, point2, [0,0,255], 3)
            time.sleep(0.2)
        temp_image = img[::-1,:,:]
        i += 1
        if value == 1:
            cv2.imshow("Demo", temp_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break



