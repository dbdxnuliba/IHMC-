import numpy as np
import cv2

cam1 = cv2.VideoCapture(1)
cam2 = cv2.VideoCapture(2)

while(True):
    # Capture frame-by-frame
    ret1, frame1 = cam1.read()
    ret2, frame2 = cam2.read()

    # Our operations on the frame comes here
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame1',gray1)
    cv2.imshow('frame2',gray2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything's done, release the capture
cam1.release()
cam2.release()
cv2.destroyAllWindows()
