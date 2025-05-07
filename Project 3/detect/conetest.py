import cv2
import numpy as np

for k in range(0, 9):
    image = cv2.imread(f'Project 3/detect/coneEqn/capture_{k}.jpg')
    coneValues = np.array([[0, 88, 47], [255, 255, 255]])

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masked = cv2.inRange(hsv, coneValues[0], coneValues[1])

    pixelCount = 0
    for i in range(0, image.shape[0]):
        for j in range(0, image.shape[1]):
            if masked[i, j] == 255:
                pixelCount += 1

    print(pixelCount)

    # cv2.imshow("masked", masked)
    # cv2.waitKey(0)