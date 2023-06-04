import cv2

webcam = cv2.VideoCapture(0)

print("Vamos começar")

try:
    if webcam.isOpened():
        validacao, frame = webcam.read()
        while validacao:
            validacao, frame = webcam.read()
            print(frame)
            # cv2.imshow("Camera log",frame)
            key = cv2.waitKey(2)
except KeyboardInterrupt:
    webcam.release()
    print('FIM')