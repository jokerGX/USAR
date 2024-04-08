import cv2
import numpy as np
from collections import Counter

def capture_image_from_camera(filename='test.jpg'):
    cam = cv2.VideoCapture(0)
    cam.set(3, 640)  # Width
    cam.set(4, 480)  # Height
    ret, frame = cam.read()
    if ret:
        frame = cv2.flip(frame, -1)  # Flips the frame vertically and horizontally
        cv2.imwrite(filename, frame)
    cam.release()
    return ret

def determine_arrow_direction(box_points, centroid):
    vectors = box_points - centroid
    distances = np.linalg.norm(vectors, axis=1)
    tip_index = np.argmax(distances)
    tail_index = (tip_index + 2) % 4
    direction_vector = vectors[tip_index] - vectors[tail_index]
    direction_vector = direction_vector / np.linalg.norm(direction_vector)
    angle = np.arctan2(direction_vector[1], direction_vector[0])
    if -np.pi / 2 <= angle <= np.pi / 2:
        return 'left'
    else:
        return 'right'

def find_arrow_direction(image_path):
    # Load the image
    img = cv2.imread(image_path)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # light condition
    # lower_blue = np.array([100, 70, 60])
    # upper_blue = np.array([140, 160, 110])

    # dark condition
    lower_blue = np.array([100, 80, 70])
    upper_blue = np.array([130, 180, 150])
    
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    cv2.imwrite('mask.jpg', mask)

    # Noise reduction with Gaussian Blur
    blurred = cv2.GaussianBlur(mask, (5, 5), 0)

    # Morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    opening = cv2.morphologyEx(blurred, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, hierarchy = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    # Filter contours based on area and find the largest one
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.imwrite('contours.jpg', cv2.drawContours(img, [box], 0, (0, 0, 255), 2))

        # Calculate the centroid of the contour
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centroid = (cX, cY)

            # Determine the direction of the arrow
            return determine_arrow_direction(box, centroid)
    return "none"

def get_most_probable_direction(num_samples=8):
    directions = []
    for i in range(num_samples):
        if capture_image_from_camera('test.jpg'):
            direction = find_arrow_direction('test.jpg')
            directions.append(direction)
        else:
            print("Failed to capture image.")

    direction_counter = Counter(directions)
    most_common = direction_counter.most_common(1)
    if most_common:
        return most_common[0][0]  # Return the most common direction
    else:
        return "none"
    

# if __name__ == "__main__":
#     direction = get_most_probable_direction()
#     print(direction)


