import cv2
from pyfirmata import Arduino, SERVO
from time import sleep
import mediapipe as mp

face = mp.solutions.face_detection
Face = face.FaceDetection()
mpDwaw = mp.solutions.drawing_utils

port = 'COM9'
pinH = 8
pinV = 10
board = Arduino(port)

board.digital[pinH].mode = SERVO
board.digital[pinV].mode = SERVO

dt = 1e-3

def rotateServo(pin,angle):
    board.digital[pin].write(angle)
    sleep(dt)

cap = cv2.VideoCapture(0)

positionX = 90
positionY = 120

board.digital[pinV].write(0)

rotateServo(pinH, positionX)
rotateServo(pinV, positionY)

Kp = 1.5e-2
Ki = 2e-1
Kd = 3.8e-6
lastError = 0
error = 0
integral = 0
output = 0

lastErrorY = 0
errorY = 0
integralY = 0
outputY = 0


while True:
    ret, img = cap.read()
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = Face.process(imgRGB)
    facesPoints = results.detections
    hO, wO, _ = img.shape

    if facesPoints:
        for id, detection in enumerate(facesPoints):
            bbox = detection.location_data.relative_bounding_box
            x,y,w,h = int(bbox.xmin*wO),int(bbox.ymin*hO),int(bbox.width*wO),int(bbox.height*hO)

            cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            
            xx = int(x + (x + w)) // 2
            yy = int(y + (y + h)) // 2

            ctX = int(wO / 2)
            ctY = int(hO / 2)
            
            setPoint = ctX
            lastError = error
            error = setPoint - xx
            
            proportional = Kp * error
            integral += Ki * error * dt
            derivative = Kd * (error - lastError) / dt
            
            pid = proportional + integral + derivative
            output = int(pid) 
            
            positionX += output
            
            if positionX > 180:
                positionX = 180
            elif positionX < 0:
                positionX = 0
            
            rotateServo(pinH, positionX)
            
            setPointY = ctY
            lastErrorY = errorY
            errorY = setPointY - yy
            
            proportionalY = Kp * errorY
            integralY += Ki * errorY * dt
            derivativeY = Kd * (errorY - lastErrorY) / dt
            
            pidY = proportionalY + integralY + derivativeY
            outputY = int(pidY) 
            
            positionY -= outputY
            
            if positionY > 180:
                positionY = 180
            elif positionY < 0:
                positionY = 0
                
            rotateServo(pinV, positionY)
            
            break

    flipedImg = cv2.flip(img, flipCode=1)
    cv2.imshow('img', flipedImg)

    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break
