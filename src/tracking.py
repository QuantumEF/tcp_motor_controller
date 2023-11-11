# import the necessary packages
import numpy as np
import cv2
import time
import serial
 
face_classifier = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)
print("Hello")

cv2.startWindowThread()

cap = cv2.VideoCapture(2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

def scale_number(unscaled, to_min, to_max, from_min, from_max):
    return (to_max-to_min)*(unscaled-from_min)/(from_max-from_min)+to_min

ser = serial.Serial('/dev/ttyACM0',baudrate=115200)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    frame = frame[0:1280,0:960]
    # resizing for faster detection
    frame = cv2.resize(frame, (640, 480))
    # using a greyscale picture, also for faster detection
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    boxes = face_classifier.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40)
    )

    com = np.array([[x + (w//2), y+(h//2)] for (x, y, w, h) in boxes])

    if len(com) == 0:
        ser.write(f'{0}\n'.encode())
        
    for (x, y) in com:
        scaled = scale_number(x, -1333, 1333, 0, 640)
        ser.write(f'{int(scaled)}\n'.encode())
        print(f'{int(scaled)}\n'.encode())
        # display the detected boxes in the colour picture
        cv2.circle(frame, (x, y) ,100, (0, 255, 0), 2)
    
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        ser.write(f'{0}\n'.encode())
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
cv2.waitKey(1)