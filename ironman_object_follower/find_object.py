#!/Users/marilyn/anaconda3/envs/cs7785-l1/bin/python

# Marilyn Braojos 
# Mariam Misabishvili

import cv2
import numpy as np

def find_object():

    cap = cv2.VideoCapture(0)  # 0 \eq default camera; cap = vid capture stream
    
    # error message: if camera not accessible
    if not cap.isOpened():
        print("Error: Camera is not found.")
        return

    # color range: https://stackoverflow.com/questions/36817133/identifying-the-range-of-a-color-in-hsv-using-opencv
    # http://www.workwithcolor.com/green-color-hue-range-01.htm
    # https://stackoverflow.com/questions/47483951/how-can-i-define-a-threshold-value-to-detect-only-green-colour-objects-in-an-ima 
    lower_color = np.array([40, 75, 75])
    upper_color = np.array([80, 255, 255])

    while(True):

        # https://docs.opencv.org/3.4/d8/dfe/classcv_1_1VideoCapture.html 
        ret, frame = cap.read() # returns video frame
        # ret = boolean indicating if frame was read
        # frame = the image captured

        # error message: if frame not captured
        if not ret:
            print("Failed to grab frame.")
            break

        # convert frame captured to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # check if elements are within the specified color range and creates binary mask 
        bin_mask = cv2.inRange(hsv_frame, lower_color, upper_color)

        # find contours (args: src img, contour retr mode, contour appx)
        # https://docs.opencv.org/4.x/d4/d73/tutorial_py_contours_begin.html
        contours, _ = cv2.findContours(bin_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # for all contours found
        # https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 2000:  # filter area of objects to detect
                continue

            x, y, w, h = cv2.boundingRect(contour)

            mask_roi = bin_mask[y:y+h, x:x+w]
            avg_intensity = np.mean(mask_roi) / 255.0

            # if the contour's avg intensity is a shade of blk on average, go to next contour
            if avg_intensity < 0.35:
                continue

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"Object Detected at Position: ({x}, {y})", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # display frame
        cv2.imshow("Object Tracking Operation", frame)

        # exit by pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # release camera and close the window
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    find_object()