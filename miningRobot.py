from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
import sys
import select
import imutils
import client
import maestro


def brighten(img, value = 70):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

##def sendCommand(x):
##    if(x == '8'):
##        tango.setTarget(MOTOR, 6800)
class KeyControl():
    def __init__(self,win):
        self.root = win
        self.tango = maestro.Controller()
        self.body = 6000
        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000
        self.shoulder = 6000
        self.wrist = 6000
        self.elbow = 6000
        self.hand = 6000


    def head(self,key):
        if key == 'u':
            self.headTilt = 5500
            self.headTurn = 6000
            self.tango.setTarget(HEADTURN, self.headTurn)
            self.tango.setTarget(HEADTILT, self.headTilt)
        elif key == 'm':
            self.headTilt = 5500
            self.headTurn = 6000
            self.tango.setTarget(HEADTILT, self.headTilt)
            self.tango.setTarget(HEADTURN, self.headTurn)
        elif key == 'd':
            self.headTilt = 1510
            self.headTurn = 6000
            self.tango.setTarget(HEADTURN, self.headTurn)
            self.tango.setTarget(HEADTILT, self.headTilt)
        elif key == 'l':
            self.headTurn = 6200
            self.headTilt = 1510
            self.tango.setTarget(HEADTILT, self.headTilt)
            self.tango.setTarget(HEADTURN, self.headTurn)

    def arm(self,key):
        if key == 'e1':     #elbow up
            self.elbow = 3400
            if(self.elbow > 7900):
                self.elbow = 7900
            self.tango.setTarget(ELBOW, self.elbow)
        elif key == 'e2':   #elbow down
            self.elbow = 7000
            if(self.elbow < 1510):
                self.elbow = 1510
            self.tango.setTarget(ELBOW, self.elbow)
        elif key == 'h1':   #hand close
            self.hand = 6000
            if(self.hand > 7900):
                self.hand = 7900
            self.tango.setTarget(HAND, self.hand)
        elif key == 'h2':   #hand open
            self.hand = 1510
            if(self.hand < 1510):
                self.hand = 1510
            self.tango.setTarget(HAND, self.hand)
        elif key == 'sh1':      #shoulder up
            self.shoulder = 5500
            if(self.shoulder > 7900):
                self.shoulder = 7900
            self.tango.setTarget(SHOULDER, self.shoulder)
        elif key == 'sh2':      #should down
            self.shoulder = 7900
            if(self.shoulder < 1510):
                self.shoulder = 1510
            self.tango.setTarget(SHOULDER, self.shoulder)
        elif key == 'sh3':      #should down
            self.shoulder = 7000
            if(self.shoulder < 1510):
                self.shoulder = 1510
            self.tango.setTarget(SHOULDER, self.shoulder)



    def arrow(self,key):
        print(key)
        if key == 'f':
            self.turn = 6000
            self.motors = 5200
            if(self.motors > 7900):
                self.motors = 7900
            print(self.motors)
            self.tango.setTarget(MOTORS, self.motors)
            self.tango.setTarget(TURN, self.turn)
        elif key == 'b':
            self.turn = 6000
            self.motors = 6800
            if(self.motors > 7900):
                self.motors = 7900
            print(self.motors)
            self.tango.setTarget(MOTORS, self.motors)
            self.tango.setTarget(TURN, self.turn)
        elif key == 'l':
            self.motors = 6000
            self.turn = 4900
            if(self.turn > 7400):
                self.turn = 7400
            print(self.turn)
            self.tango.setTarget(MOTORS, self.motors)
            self.tango.setTarget(TURN, self.turn)
        elif key == 'r':
            self.turn = 7100
            self.motors = 6000
            if(self.turn <2110):
                self.turn = 2110
            print(self.turn)
            self.tango.setTarget(MOTORS, self.motors)
            self.tango.setTarget(TURN, self.turn)

        elif key == 's':
            self.motors = 6000
            self.turn = 6000
            self.tango.setTarget(MOTORS, self.motors)
            self.tango.setTarget(TURN, self.turn)

def findMarker():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    time.sleep(1)

    win = "Frame"
    keys = KeyControl(win)

    keys.head('d')
    found = False
    warmUp = 0

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hsv = hsv[200:480,220:380]

        lowerOrange = np.array([10,50,50])
        upperOrange = np.array([45,255,255])

        orangeMask = cv2.inRange(hsv, lowerOrange, upperOrange)
        orangePath = np.argwhere(orangeMask != 0)

        print(len(orangePath))
        if len(orangePath) >= 9000:
            found = True
            #keys.arrow('l')
            time.sleep(.5)

        mean = np.mean(orangePath,axis=0)
        floor = mean.astype(int)
        y = floor[0]
        x = floor[1]
        print(x)

        cv2.imshow('Image',img)
        cv2.imshow('Mask',orangeMask)
        cv2.imshow('HSV', hsv)
        rawCapture.truncate(0)
        if warmUp <= 15:
            warmUp += 1
        else:
            if found == False:
                keys.arrow('r')
                time.sleep(.5)
                keys.arrow('s')
            else:
                if x >= 70 and x <= 90:
                    break
                if x < 70:
                    keys.arrow('r')
                    time.sleep(.25)
                    keys.arrow('s')
                if x > 80:
                    keys.arrow('l')
                    time.sleep(.25)
                    keys.arrow('s')

        key = cv2.waitKey(1) & 0xFF

        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if line[0]=="q":
                keys.arrow('s')
                exit()
    camera.close()

def findMarker1():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    time.sleep(1)

    win = "Frame"
    keys = KeyControl(win)

    keys.head('d')
    found = False
    warmUp = 0

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hsv = hsv[150:480,220:380]

        lowerOrange = np.array([15,50,50])
        upperOrange = np.array([45,255,255])

        orangeMask = cv2.inRange(hsv, lowerOrange, upperOrange)
        orangePath = np.argwhere(orangeMask != 0)

        print(len(orangePath))
        if len(orangePath) >= 4000 and len(orangePath) <= 12000:
            found = True
            #keys.arrow('r')
            time.sleep(.75)

        mean = np.mean(orangePath,axis=0)
        floor = mean.astype(int)
        y = floor[0]
        x = floor[1]
        print(x)

        cv2.imshow('Image',img)
        cv2.imshow('Mask',orangeMask)
        cv2.imshow('HSV', hsv)
        rawCapture.truncate(0)
        if warmUp <= 15:
            warmUp += 1
        else:
            if found == False:
                keys.arrow('r')
                time.sleep(.75)
                keys.arrow('s')
            else:
                if x >= 70 and x <= 90:
                    break
                if x < 70:
                    keys.arrow('r')
                    time.sleep(.25)
                    keys.arrow('s')
                if x > 80:
                    keys.arrow('l')
                    time.sleep(.25)
                    keys.arrow('s')

        key = cv2.waitKey(1) & 0xFF

        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if line[0]=="q":
                keys.arrow('s')
                exit()
    camera.close()

def rockField():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    kernel = np.ones((5,5), np.uint8)

    # allow the camera to warmup
    time.sleep(1)

    win = "Frame"
    keys = KeyControl(win)

    #keys.head('u')
    keys.head('d')
    pastFirstLine = False
    inRocks = False
    frameTime = 0

    client.sendData("Entering Rock Field")

    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        pic = frame.array
        hsv = cv2.cvtColor(pic, cv2.COLOR_BGR2HSV)

        #hsv = cv2.blur(hsv, (5,5))
        hsv = hsv[300:480,160:480]

        lowerOrange = np.array([10,50,50])
        upperOrange = np.array([35,255,255])

        orangeMask = cv2.inRange(hsv, lowerOrange, upperOrange)
        orangePath = np.argwhere(orangeMask != 0)


        lowerWhite = np.array([100,0,200])
        upperWhite = np.array([180,135,255])
        xLowWhite = np.array([0,0,200])
        xUpperWhite = np.array([10,135,255])

        whiteMask1 = cv2.inRange(hsv, lowerWhite, upperWhite)
        whiteMask2 = cv2.inRange(hsv, xLowWhite, xUpperWhite)
        whiteMask = whiteMask1 + whiteMask2
        whitePath = np.argwhere(whiteMask != 255)

        cv2.imshow("HSV", hsv)
        cv2.imshow("Frame", whiteMask)



            #keys.arrow("f")
            #time.sleep(1.5)
            #print(mask)
        #cv2.imshow("HSV", mask1)
        #cv2.imshow("HSV1", hsv)

        #pic = brighten(pic)
        # pic = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
        # ret,thresh = cv2.threshold(pic,127,255,0)
        # pic = cv2.blur(pic, (3,3))
        # pic = pic[420:480,160:480]
        # pic = cv2.Canny(pic, 100, 170)
        # pic = cv2.dilate(pic, kernel, iterations=5)
        # pixels = np.argwhere(pic == 0)
        orangeMean = np.mean(orangePath,axis=0)
        orangeFloor = orangeMean.astype(int)
        orangeY = orangeFloor[0]
        orangeX = orangeFloor[1]

        whiteMean = np.mean(whitePath,axis=0)
        whiteFloor = whiteMean.astype(int)
        whiteY = whiteFloor[0]
        whiteX = whiteFloor[1]
        mid = 160
        #print(whiteX)

        #cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)

        key = cv2.waitKey(1) & 0xFF


        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if line[0]=="q":
                keys.arrow('s')
                exit()
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        #checks for orange at bottom of camera for first line
        #print(pastFirstLine, frameTime)
        #print(inRocks)
        pastFirstLine = True

        #Checks if first line is crossed, if so starts short timer
        if pastFirstLine == True:
            frameTime += 1
            if frameTime >= 60: #when timer reaches 15, declares in rockfield
                inRocks = True
            if inRocks == True: #if in rock field
                #print("True:", orangeY)
                if orangeY >= 120:  #if next line in about to be crossed then break function
                    cv2.destroyAllWindows()
                    break

        #print(whiteX)
        if whiteX >= 155 and whiteX <= 165:
            keys.arrow('f')
        if whiteX < 155:
            keys.arrow('r')
        if whiteX > 165:
            keys.arrow('l')
    camera.close()

def findHuman():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    time.sleep(1)

    win = "Frame"
    keys = KeyControl(win)

    keys.arrow('f')
    time.sleep(.25)
    client.sendData("Entering Mining Zone")

    keys.head('u')
    found = False

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        width = 0
        for (x,y,w,h) in faces:
            found = True
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0))
            centerX=x+(w/2)
            centerY=y+(h/2)
            width = w
            size=w*h

        rawCapture.truncate(0)

        print(found)
        if found == False: #if not found, search room for face
            keys.arrow('r')
            time.sleep(.5)
            keys.arrow('s')
        else:
            print(centerX)
            if centerX >= 280 and centerX <= 360: #if person is centered then go forward
                print("Width:", width)
                if width >= 100: #if close enough, then break out of function
                    keys.arrow('s')
                    client.sendData("Give me ice")
                    cv2.destroyAllWindows()
                    break
                else: #else go forward
                    print(w)
                    keys.arrow('f')
                    time.sleep(1)
                    keys.arrow('s')
            if centerX < 280:   #center face
                keys.arrow('r')
                time.sleep(.5)
                keys.arrow('s')
            if centerX > 360:
                keys.arrow('l')
                time.sleep(.5)
                keys.arrow('s')

        cv2.imshow('Image',img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        centerX = 160
        time.sleep(.5)

    camera.close()

def getIce():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    time.sleep(1)

    win = "Frame"
    keys = KeyControl(win)

    keys.head('l')
    keys.arm('sh1')
    keys.arm('h2')
    frameCount = 0
    frameCountOther = 0



    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        found = False
        img = frame.array

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #hsv = hsv[300:480,160:480] #make screen smaller

        lowerPink = np.array([150,0,200])
        upperPink = np.array([165,135,255])

        pinkMask = cv2.inRange(hsv, lowerPink, upperPink)
        pinkPath = np.argwhere(pinkMask != 0)

        if len(pinkPath) >= 500:
            found = True
        else:
            found = False

        if found == True:
            frameCount += 1
            frameCountOther = 0
        else:
            frameCount = 0
            frameCountOther +=1

        if frameCount >= 60:
            client.sendData("Thank you for the pink ice")
            keys.arm('h1')
            time.sleep(1.5)
            keys.arm('sh2')
            keys.arm('u')
            time.sleep(.5)
            break

        if frameCountOther >= 30:
            client.sendData("I Want the pink ice")
            frameCountOther = 0

        key = cv2.waitKey(1) & 0xFF
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        cv2.imshow('Image',pinkMask)
        cv2.imshow("HSV", hsv)

        rawCapture.truncate(0)
    camera.close()

def getBuckets():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))


    time.sleep(1)

    win = "Frame"
    keys = KeyControl(win)

    keys.arrow('f')
    time.sleep(.1)

    keys.head('d')
    keys.arm('sh3')
    found = False

    client.sendData("Entering Score Zone")

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hsv = hsv[0:480,160:480]

        lowerOrange = np.array([150,0,200])
        upperOrange = np.array([165,135,255])

        orangeMask = cv2.inRange(hsv , lowerOrange, upperOrange)
        orangePath = np.argwhere(orangeMask != 0)

        print(len(orangePath))
        if len(orangePath) >= 500:
            found = True

        mean = np.mean(orangePath,axis=0)
        floor = mean.astype(int)
        y = floor[0]
        x = floor[1]

        rawCapture.truncate(0)

        if found == False: #if not found, search room for bucket
            keys.arrow('l')
            time.sleep(.5)
            keys.arrow('s')
        else:
            if x >= 150 and x <= 170: #if bucket is centered then go forward
                if y >= 300: #if close enough, then make bucket
                    keys.arrow('f')
                    time.sleep(1.75)
                    keys.arrow('s')
                    keys.arrow('l')
                    time.sleep(.75)
                    keys.arrow('s')
                    keys.arrow('h2')
                    print("letgo")
                    time.sleep(2)
                    cv2.destroyAllWindows()
                    break
                else: #else go forward
                    keys.arrow('f')
                    time.sleep(1)
                    keys.arrow('s')
            if x < 150:   #center bucket
                keys.arrow('r')
                time.sleep(.25)
                keys.arrow('s')
            if x > 170:
                keys.arrow('l')
                time.sleep(.25)
                keys.arrow('s')

        cv2.imshow("HSV", hsv)
        cv2.imshow('Image',orangeMask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    MOTORS = 1
    TURN = 2
    BODY = 0
    HEADTILT = 4
    HEADTURN = 3
    SHOULDER = 12
    WRIST = 17
    ELBOW = 15
    HAND = 13

    IP = '10.200.46.186'
    PORT = 5010
    client = client.ClientSocket(IP, PORT)


    win = "Frame"
    keys = KeyControl(win)

    keys.arm("e2")
    keys.arm("h2")
    keys.arm("sh2")



    findMarker1()
    rockField()
    findHuman()
    getIce()
    findMarker()
    rockField()
    getBuckets()

    keys.arm("h2")
    time.sleep(1)

    keys.arrow('s')
    exit()
