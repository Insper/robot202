import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

cap = cv2.VideoCapture('line_following.mp4')

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    #channel_count = img.shape[2]
    match_mask_color = 255#(255,)*channel_count
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_the_lines(img, lines):
    img_copy = np.copy(img)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    lines_dic = dict()
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope, intercept = calc_slope_intercept(x1, y1, x2, y2)
            if (slope < -0.31) or (slope > 0.43):
                lines_dic[slope] = line
    
    lista_slopes = list(lines_dic.keys())
    try:
        min_slope = min(lista_slopes)
        max_slope = max(lista_slopes)
        lines = [lines_dic[min_slope], lines_dic[max_slope]]
    except:
        lines = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(img_copy, (x1,y1), (x2,y2), (0,255,0), thickness=3)
            
    img_copy = cv2.addWeighted(img_copy, 0.8, line_img, 1, 0.0)
    return img_copy, lines

def calc_slope_intercept(x1, y1, x2, y2):
    slope = (y2-y1)/(x2-x1)
    intercept = (y1 - slope*x1)
    return slope, intercept

def draw_escape_point(img, lines):
    img_copy = np.copy(img)
    point_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    try:
        a1, b1 = calc_slope_intercept(lines[0][0][0],lines[0][0][1],lines[0][0][2],lines[0][0][3])
        a2, b2 = calc_slope_intercept(lines[1][0][0],lines[1][0][1],lines[1][0][2],lines[1][0][3])
        x = (b2-b1)/(a1-a2)
        y = a1*x + b1
        x, y = int(np.round(x)), int(np.round(y))
        cv2.circle(img_copy, (x,y), 2, (0,255,0), thickness=3)
        img_copy = cv2.addWeighted(img_copy, 0.8, point_img, 1, 0.0)
        return img_copy
    except:
        return img_copy
    
while (cap.isOpened()):

    ret, frame = cap.read()
    
    height = frame.shape[0]
    width = frame.shape[1]
    region_of_interest_vertices = [
        (10, height), (10, height/2), (width-10, height), (width-10, height/2)
    ]
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 100, 150)
    
    masked_img = region_of_interest(canny, np.array([region_of_interest_vertices], np.int32))
    
    lines = cv2.HoughLinesP(masked_img, rho=6, theta=np.pi/60, threshold=160, lines=np.array([]), minLineLength=120, maxLineGap=25)
    
    output, lines = draw_the_lines(frame, lines)
    output = draw_escape_point(output, lines)
    
    cv2.imshow('frame', output)
    
    #time.sleep(0.1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()