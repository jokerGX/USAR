from __future__ import print_function
import cv2
import numpy as np
import robot 
import time
from cv import get_most_probable_direction
TARGET_LOCKED = False
last_known_ball_position = None

def find_orange_ball(frame, direction, decide):
    global last_known_ball_position
    global TARGET_LOCKED

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if decide == 0:
        lower_orange = np.array([150, 140, 50])
        upper_orange = np.array([195, 239, 150])
    elif decide == 1:
        lower_orange = np.array([160, 100, 90])
        upper_orange = np.array([190, 230, 230])

    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
    frame_center = frame.shape[1] // 2

    valid_contours = []
    for c in contours:
        M = cv2.moments(c)
        if M["m00"] == 0: continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        radius = cv2.minEnclosingCircle(c)[1]
        valid_contours.append((c, cx, cy, radius))

    if valid_contours:
        if direction == "left" or direction == "right":
            frame_center = frame.shape[1] // 2
            if direction == "left":
                ball, cx, cy, radius = min(valid_contours, key=lambda x: x[1])
            else:  
                ball, cx, cy, radius = max(valid_contours, key=lambda x: x[1])
        else:
            if last_known_ball_position is not None:
                ball, cx, cy, radius = min(valid_contours, key=lambda x: np.linalg.norm(np.array(last_known_ball_position) - np.array([x[1], x[2]])))
            else:
                ball, cx, cy, radius = valid_contours[0]
        last_known_ball_position = (cx, cy)
        TARGET_LOCKED = True
        return (cx, cy), radius, mask
    else:
        return None, None, mask

def cv_auto(direction):
    global TARGET_LOCKED 
    robot_instance = robot.Robot()
    cap = cv2.VideoCapture(0)

    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (frame_width, frame_height))

    flag = False

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture image")
                break

            frame = cv2.flip(frame, -1)
            mask = np.zeros_like(frame)
            
            if not flag:
                center, radius, mask = find_orange_ball(frame, direction if not TARGET_LOCKED else "", 0)
                if radius > 30:
                    flag = True
            elif flag:
                center, radius, mask = find_orange_ball(frame, direction if not TARGET_LOCKED else "", 1)



            if center and radius > 10: 
                mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                cv2.circle(mask_colored, center, int(radius), (0, 255, 0), 2)
                
                
                print("Ball found, moving towards it...")
                frame_center = frame.shape[1] // 2 
                if center[0] < frame_center :
                    robot_instance.set_robot_power(-20, -60) 
                elif center[0] > frame_center:  
                    robot_instance.set_robot_power(-60, -20) 
                else:
                    robot_instance.set_robot_power(-40, -40) 
                if radius >= 48: 
                    print("max distance")
                    robot_instance.set_robot_power(-50, -50)
                    print("put arm down")
                    robot_instance.set_arm_power(13.5)
                    time.sleep(0.5)
                    robot_instance.set_arm_power(0)
                    time.sleep(1.2)
                    robot_instance.set_robot_power(0, 0) 
                    print("move forward")
                    robot_instance.set_robot_power(50, -50)  
                    time.sleep(1.5)
                    robot_instance.set_robot_power(-50, 50)  
                    time.sleep(3)
                    print("Ball reached")

                    robot_instance.set_robot_power(0, 0) 
                    break
            else:
                print("Searching for the ball...")
                robot_instance.set_robot_power(50, -50) 
            # out.write(frame)
            out.write(mask_colored)
            # time.sleep(0.1)
    finally:
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        robot_instance.reset_all()


