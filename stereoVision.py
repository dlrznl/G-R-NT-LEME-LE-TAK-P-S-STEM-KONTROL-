from code import interact
from re import X
import sys
from turtle import delay
import cv2
import numpy as np
import time
import imutils
from matplotlib import pyplot as plt
import serial
import sys
import cv2


# derinlik ve kalibrasyon 
import triangulation as tri #DERİNLİK
import calibration #STEREO MATRİS 


# yüz tanıma kütüphanesi
import mediapipe as mp
import time

# ARDUINO 
ser = serial.Serial("COM4", '9600', timeout=2)
Xposition =0
Yposition = 0


#############################3d points#####################
import cv2
import numpy as np




#############################3d points#####################


# Stereo vision setup parameters
frame_rate = 120    #KAMERA FRAME ORANI
B = 12               #KAMERA ARAS MESAFE - CM
f = 8              #KAMERA LENS ODAK UZAKLIĞI[mm]
alpha = 56.6        #Yatay düzlemde kamera görüş alanı [derece]


mp_facedetector = mp.solutions.face_detection
mp_draw = mp.solutions.drawing_utils

# KAMERALAR AKTİF
cap_right = cv2.VideoCapture(1, cv2.CAP_DSHOW)                    
cap_left =  cv2.VideoCapture(0, cv2.CAP_DSHOW)




# STEREO KAMERALARA YÜZ TANIMA VE DERİNLİK ENTEGRASYONU
with mp_facedetector.FaceDetection(min_detection_confidence=0.7) as face_detection:

    while(cap_right.isOpened() and cap_left.isOpened()):

        succes_right, frame_right = cap_right.read()
        succes_left, frame_left = cap_left.read()

    
        frame_right, frame_left = calibration.undistortRectify(frame_right, frame_left)

        if not succes_right or not succes_left:                    
            break

        else:

            start = time.time()
            
            # BGR ->  RGB
            frame_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2RGB)
            frame_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2RGB)

            # RESİMDE YÜZ TANIMA
            results_right = face_detection.process(frame_right)
            results_left = face_detection.process(frame_left)

            # RGB->BGR
            frame_right = cv2.cvtColor(frame_right, cv2.COLOR_RGB2BGR)
            frame_left = cv2.cvtColor(frame_left, cv2.COLOR_RGB2BGR)

            
            #ELDE EDİLEN GÖRÜNTÜNÜN ÖLÇEKLENDİRİLMESİ
            scale_percent = 50

            width1 = int(frame_left.shape[1] * scale_percent / 100)
            height1 = int(frame_left.shape[0] * scale_percent / 100)

            width2 = int(frame_right.shape[1] * scale_percent / 100)
            height2 = int(frame_right.shape[0] * scale_percent / 100)

            # dsize
            dsize1 = (width1, height1)
            dsize2 = (width2, height2)

            output1 = cv2.resize(frame_left, dsize1)
            output2 = cv2.resize(frame_right, dsize2)

            frame0_new=cv2.cvtColor(output1, cv2.COLOR_BGR2GRAY)
            frame1_new=cv2.cvtColor(output2, cv2.COLOR_BGR2GRAY)


            stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
            disparity = stereo.compute(frame0_new,frame1_new)

            #KALİBRASYON PARA

            disparity.shape

            Q2 =[[ 1,  0,  0,  -317.4203],
                [ 0,  1,  0,  -175.5213],
                [ 0,  0,   0,   494.7823],
                [ 1,  0,  -50,  2529.26]]


            Q1 = np.array(Q2,dtype=object)
            Q= Q1.astype(np.uint8)
            threeDImage = cv2.reprojectImageTo3D(disparity, Q)
            points = threeDImage
            points=list(points)

            
            ################## DERİNLİK #########################################################
            center_right = 0
            center_left = 0


            if results_right.detections:
                for id, detection in enumerate(results_right.detections):
                    mp_draw.draw_detection(frame_right, detection)

                    bBox = detection.location_data.relative_bounding_box

                    h, w, c = frame_right.shape
                   
                    boundBox = int(bBox.xmin * w), int(bBox.ymin * h), int(bBox.width * w), int(bBox.height * h)
                    #print(boundBox)
                    center_point_right = (boundBox[0] + boundBox[2] / 2, boundBox[1] + boundBox[3] / 2)
                    
                    #print(center_point_right)
                    cv2.putText(frame_right, f'{int(detection.score[0]*100)}%', (boundBox[0], boundBox[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0), 2)
                    
                    
                    #X Y KOORDİNATLARININ TESPİTİ

                    medium_x=boundBox[0] + boundBox[2] / 2
                    #print(medium_x)
                    delay(1000)
                    medium_y=boundBox[1] + boundBox[3] / 2
                    stepSize=1

                    coordinateinpixel_x = threeDImage[int(medium_x/2.7)][int(medium_y/2.5)][0]
                    coordinateinpixel_y = threeDImage[int(medium_x/2.7)][int(medium_y/2.5)][1]
                    print("x coor", medium_x, width1)
                    print("Y coor", medium_y, height1)

                    #ARDUINOYA VERİNİN AKTARILMASI
                    data = "X{0:d}Y{1:d}Z".format( int(medium_x), int(medium_y))
                    print ("output = '" +data+ "'")
                    ser.write((data).encode('utf-8'))
                                           
    
                                                                                




            if results_left.detections:
                for id, detection in enumerate(results_left.detections):
                    mp_draw.draw_detection(frame_left, detection)

                    bBox = detection.location_data.relative_bounding_box

                    h, w, c = frame_left.shape

                    boundBox = int(bBox.xmin * w), int(bBox.ymin * h), int(bBox.width * w), int(bBox.height * h) 
                    center_point_left = (boundBox[0] + boundBox[2] / 2, boundBox[1] + boundBox[3] / 2)
          
                    medium_x=boundBox[0] + boundBox[2] / 2
                    medium_y=boundBox[1] + boundBox[3] / 2

  
                    cv2.putText(frame_left, f'{int(detection.score[0]*100)}%', (boundBox[0], boundBox[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0), 2)


                           
                                                                         
           
                   

            if not results_right.detections or not results_left.detections:
                cv2.putText(frame_right, "TRACKING LOST", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)
                cv2.putText(frame_left, "TRACKING LOST", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)



            end = time.time()
            totalTime = end - start

            fps = 1 / totalTime
            #print("FPS: ", fps)

            cv2.putText(frame_right, f'FPS: {int(fps)}', (20,450), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 2)
            cv2.putText(frame_left, f'FPS: {int(fps)}', (20,450), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 2)    
          
                                


            # Show the frames
            cv2.imshow("frame right", frame_right) 
            cv2.imshow("frame left", frame_left)


            # Hit "q" to close the window
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


cv2.destroyAllWindows()