import sys
sys.path.insert(0,"../")
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import imutils
import maestro
import select
import client
import threading
import random


#the tango bot drives and waits on timer threads so we can still capture frames
#without waiting on wheel movement to take place
class Timer(threading.Thread):
    def __init__(self, seconds):
        super(Timer, self).__init__()
        self.seconds=seconds
    def run(self):
        time.sleep(self.seconds)


#class RobotDriver sets motor speed of the tango bot
class RobotDriver():
    def __init__(self, b, turn, tilt, m, bodyTurn, arm=6, hand=11):
        self.tango = maestro.Controller()
        self.body = 6000
        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000
        self.arm = 6000
        self.hand = 6000
        self.motorList=[m, bodyTurn, b, tilt, turn, arm, hand]
        self.motorValues=[self.motors, self.turn, self.body, self.headTilt, self.headTurn, self.arm, self.hand]
        self.timer = Timer(.5)
        self.lastCommandTime = 0
        self.travelDir=0 #memory for if it is going in positive direction/angle or negative

        for i in range(len(self.motorList)-2):
            self.tango.setTarget(self.motorList[i], self.motorValues[i])

    def __str__(self):
        return ('BODY: '+str(self.body)+' HEADTURN: '+str(self.headTurn)+' HEADTILT: '+str(self.headTilt)+' MOTORS: '+str(self.motors)+' TURN: '+str(self.turn))

    def setBody(self, num):
        self.motorValues[0]=num
        self.tango.setTarget(self.motorList[0], self.motorValues[0])

    def setSpeed(self, speed):
        values=[800,1200,1800]
        if speed < 0:
            self.motors=6000+values[0-speed-1]
            self.motorValues=[self.motors, self.turn, self.body, self.headTilt, self.headTurn, self.arm, self.hand]
            self.tango.setTarget(self.motorList[0], self.motorValues[0])
        if speed > 0:
            self.motors=6000-values[speed-1]
            self.motorValues=[self.motors, self.turn, self.body, self.headTilt, self.headTurn, self.arm, self.hand]
            self.tango.setTarget(self.motorList[0], self.motorValues[0])
        if speed == 0:
            self.killMotors()

    def move(self,dist,velocity):
        if dist < 0.25 and dist > -0.25: #if we are within a quarter meter of our target, stop motors
            self.killMotors()
            return dist
        else:
            drive_time=dist/velocity

            DRIVE=800
            if drive_time<0: #if time ends up being negative, make it positive, but turn the other direction
                drive_time=0-drive_time
                DRIVE=0-DRIVE
            if not self.timer.isAlive(): #dont send more wait commands to motors if they are already moving
                self.lastCommandTime=time.time()
                #self.timer.join()
                print('starting thread')
                self.timer = Timer(drive_time)
                self.timer.start()
                self.motors=6000-DRIVE
                if DRIVE<0:
                    self.travelDir=-1
                else:
                    self.travelDir=1
                self.motorValues=[self.motors, self.turn, self.body, self.headTilt, self.headTurn, self.arm, self.hand]
                self.tango.setTarget(self.motorList[0], self.motorValues[0])
            newDist=(time.time()-self.lastCommandTime)*velocity
            if self.travelDir==-1:
                newDist=0-newDist
            self.lastCommandTime=time.time()
            return newDist

    def rotate(self,angle,desired,velocity):
        if desired-angle<5 and desired-angle>-5: #if we are within 5 degrees of our target, stop motors
            self.killMotors()
            return desired-angle
        else:
            rotate_time = (desired-angle)/velocity
            TURN=1000
            if rotate_time<0: #if time ends up being negative, make it positive, but turn the other direction
                rotate_time=0-rotate_time
                TURN=0-TURN
            if not self.timer.isAlive(): #dont send more wait commands to motors if they are already moving
                self.lastCommandTime=time.time()
                #timer.join()
                self.timer = Timer(rotate_time)
                self.timer.start()
                self.turn=6000+TURN
                if TURN<0:
                    self.travelDir=-1
                else:
                    self.travelDir=1
                self.motorValues=[self.motors, self.turn, self.body, self.headTilt, self.headTurn, self.arm, self.hand]
                self.tango.setTarget(self.motorList[1], self.motorValues[1])
        newAngle=(time.time()-self.lastCommandTime)*velocity
        if self.travelDir==-1:
            newAngle=0-newAngle
        self.lastCommandTime=time.time()
        return newAngle

    def killMotors(self):
        self.DRIVE = 6000
        self.headTurn = 6000
        self.headTilt = 6500
        self.motors = 6000
        self.turn = 6000
        self.motorValues=[self.motors, self.turn, self.body, self.headTilt, self.headTurn, self.arm, self.hand]
        for i in range(len(self.motorList)-2):
            #print(self.motorValues[i])
            self.tango.setTarget(self.motorList[i], self.motorValues[i])

    def tiltHead(self, pos, turn):
        if pos == 'low':
            self.headTilt = 4000  #maybe 8000?
        if pos == 'mid':
            self.headTilt = 6000
        if pos == 'high':
            self.headTilt = 7000 #0r 4600
        if turn == 'center':
            self.headTurn= 6000
        if turn == 'left':
            self.headTurn = 6800
        if turn == 'right':
            self.headTurn = 5200
        self.motorValues=[self.motors, self.turn, self.body, self.headTilt, self.headTurn, self.arm, self.hand]
        self.tango.setTarget(self.motorList[3], self.motorValues[3])
        self.tango.setTarget(self.motorList[4], self.motorValues[4])

    def moveArm(self, action):
        if action == 'extend':
            self.arm = 7900
        if action == 'retract':
            self.arm = 4000
        if action == 'close':
            self.hand = 3000
        if action == 'open':
            self.hand = 1000
        self.motorValues=[self.motors, self.turn, self.body, self.headTilt, self.headTurn, self.arm, self.hand]
        self.tango.setTarget(self.motorList[5], self.motorValues[5])
        self.tango.setTarget(self.motorList[6], self.motorValues[6])

class WhereAmI():
    def __init__(self, phone):
        self.phone=phone
        self.angle = 0
        self.x = -1
        self.y = -1
        self.fps = 0.8  #feet per second (.8fps at default)
        self.dps = 40  #degrees per second
        self.hasIce = False
        self.tasks = ['enter obstacle stage']#'find human','verify color']#['probe position','move to start','probe speed','enter obstacle stage','find human','verify color','traverse obstacles','drop payload']
        self.location = 'start' #can be start, intermediate, end
        self.commandExcecuted=False
        self.found=False
        self.timeStart = time.time()
        self.count = 0

        # initializes tangobot driver
        # Value Reminders: MOTORS = 1  TURN = 2  BODY = 0  HEADTILT = 4  HEADTURN = 3
        self.bot = RobotDriver(0, 3, 4, 1, 2)

    def getTask(self):
        if len(self.tasks)==0:
            return 0
        return self.tasks[0]

    def processTasks(self, frame):
        if len(self.tasks)==0:
            self.phone.sendData("finished all tasks")
        elif self.tasks[0]=='probe position':
            self.findStart(frame)
        elif self.tasks[0]=='enter obstacle stage':
            self.navigate(frame)
        elif self.tasks[0]=='find human':
            self.findHuman(frame)
        elif self.tasks[0]=='verify color':
            self.detectBall(frame)
        elif self.tasks[0]=='drop payload':
            self.dropPayload(frame)
        elif self.tasks[0]=='test':
            self.testSpeed()

    def findStart(self,frame):
        pass
        #rotate until panel detected
        #drive in front of panel (smaller color is far away and will dictate front)
        #center body to panel (can use neck so we dont have to parallel park)
        #make drive forward/back until panel is a certain size (1 m away)
        #if we plan on doing speed calculations, we can do them here using triangulation with the camera, distance from panel, and time
        #drive @ speed 1,2,and 3 for 1 second, calculate distance driven for each speed and that is the self.mps value (meters per second)
        #rotate @ angular velocities 1,2,and 3 for .5 second and triangulate angle multiplied by two, that is self.dps (degrees per second)
        #current pos is x=0,y=0, angle is 180 degrees
        #rotate so angle is 0
    def navigate(self,frame):

        self.bot.tiltHead('low', 'center')

        if not self.hasIce:
            crop = frame[430:480, 145:485]
            gray= cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
            mask = cv2.inRange(crop,(200,200,200),(255,255,255))  #FIX color bounds for pink line
            mask_rgb=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
            pink = crop & mask_rgb
            cv2.imshow('pink',pink)
            #if pink line detected
            for i in pink:
                if i.any()!=0:
                    self.bot.move(2,self.fps)
                    self.tasks=self.tasks[1:]
                    self.phone.sendData("Now entering the mining area")
                    time.sleep(3)
                    self.bot.killMotors()
                    return
        else:
            crop = frame[430:480, 145:485]
            gray= cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
            mask = cv2.inRange(crop,(200,200,200),(255,255,255))  #FIX color bounds for blue line
            mask_rgb=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
            blue = crop & mask_rgb
            #if blue line detected
            for i in blue:
                if i.any()!=0:
                    self.bot.move(2,self.fps)
                    self.tasks=self.tasks[1:]
                    self.phone.sendData("Back at starting area")
                    time.sleep(3)
                    self.bot.killMotors()
                    return

        gray= cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(frame,(200,200,200),(255,255,255))
        mask_rgb=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
        frame = frame & mask_rgb
        #cv2.circle(frame, (145,440), 5, (255,0,0)) #480
        #cv2.circle(frame, (485,440), 5, (255,0,0))
        crop_right = frame[430:480, 145:315]  #315 center
        crop_left = frame[430:480, 315:485]
        obsR=False
        obsL=False
        for i in crop_right:
            if i.any()!=0:
                obsR=True
                break
        for i in crop_left:
            if i.any()!=0:
                obsL=True
                break
        if obsL and obsR:
            self.bot.move(-2,self.fps)
            time.sleep(1.5)
            rando=random.randrange(-40,40)
            self.angle=self.angle+self.bot.rotate(self.angle,self.angle+rando,dps)
        elif obsL:
            self.angle=self.angle+self.bot.rotate(self.angle,self.angle-15,dps)
        elif obsR:
            self.angle=self.angle+self.bot.rotate(self.angle,self.angle+15,dps)
        else:
            self.bot.move(1,self.fps)


        cv2.imshow('binary',frame)

    def findHuman(self,frame):
        face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        self.bot.tiltHead('high', 'center')

        #found = False
        for (x,y,w,h) in faces:
            self.found = True
            self.angle = self.angle + self.bot.rotate(self.angle,self.angle,self.dps)
            #print("("+str(x)+","+str(y)+") w="+str(w)+" h="+str(h))
            if w >= 95 and w <= 130:
                self.tasks=self.tasks[1:]
                self.phone.sendData("I crave that pink ice")
                self.fount=False
            elif w < 95:
                self.bot.move(.75,self.fps)
            elif w > 130:
                self.bot.move(-.75,self.fps)

        if self.angle>=80 and not self.found:
            self.angle=self.angle+self.bot.rotate(self.angle,-90,self.dps)
        elif not self.found:
            self.angle=self.angle+self.bot.rotate(self.angle,90,self.dps)
        #print(self.angle)
        #self.bot.rotate(self.angle,)

        #cv2.imshow("findHuman",frame)


    def detectBall(self,frame):
        #Human should have already been found
        self.bot.moveArm("open")
        self.bot.moveArm("extend")
        self.bot.tiltHead("low", 'right')

        brightness=60
        frame=np.int16(frame)
        frame=frame-brightness
        frame=np.clip(frame,0,255)
        frame=np.uint8(frame)

        kernelOpen=np.ones((5,5))
        kernelClose=np.ones((20,20))
        #pink boundaries
        upperPink=np.array([180,255,255])
        lowerPink=np.array([100,100,100])
        imgHSV= cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        # create the Mask
        mask=cv2.inRange(imgHSV,lowerPink,upperPink)
        #morphology
        maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

        maskFinal=maskClose
        conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

        cv2.drawContours(frame,conts,-1,(255,0,0),3)
        #cv2.imshow('test',frame)
        #print(len(conts))
        if len(conts) >= 1:
            self.count +=1
            #print(self.count)
            self.found = True
            for i in range(len(conts)):
                x,y,w,h=cv2.boundingRect(conts[i])
        else:
            self.found = False

        if self.found == True and self.count == 16:
            self.hasIce=True
            self.bot.moveArm("close")
            time.sleep(.5)
            self.bot.moveArm("retract")
            self.bot.tiltHead("mid", 'center')
            self.tasks=self.tasks[1:]
            self.phone.sendData("Thank you for your service")

        elif self.found == False:
            self.count = 0

        else:
            pass
    def dropPayload(self,frame):
        pass


    def testSpeed(self):
        print('start test')

        self.bot.motorValues[1]=7000
        self.bot.tango.setTarget(self.bot.motorList[1], self.bot.motorValues[1])
        time.sleep(4)
        self.bot.motorValues[1]=6000
        self.bot.tango.setTarget(self.bot.motorList[1], self.bot.motorValues[1])

        '''#speed test
        self.bot.move(2,self.fps)
        for i in range(40):
            time.sleep(.1)
            self.bot.move(2-(.8*.1*i),self.fps)
            #print(i)'''
        print('end test')
        self.bot.killMotors()
        self.tasks=self.tasks[1:]

def mainLoop(camera, rawCapture,phone):

    brain=WhereAmI(phone) #main thinking process for robot, needs phone so it can communicate
    done=False
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        img = frame.array
        stage = brain.getTask()#might change style of image we show to the brain depending on the step we are on

        brain.processTasks(img)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        '''#---------quits if q is entered in terminal
        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if line[0]=="q":
                bot.killMotors()
                cv2.destroyAllWindows()
                done=True'''
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if done or stage == 0:
            break
        #-----------

def main():

    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    # allow the camera to warmup
    time.sleep(0.1)

    IP = '10.200.50.179'
    PORT = 5010
    phone = client.ClientSocket(IP, PORT)


    mainLoop(camera, rawCapture,phone)
    cv2.destroyAllWindows()

main()


'''
(use pixelfinder to get pixel coordinates)
NOTES
---------------------
When the camera is at the low tilt: (4000)
y=410 @ 1ft
y=280 @ 2ft

TODO: set default fps for body = +-800
set default dps for tilt = +-
make find color function with 6 default values (3 ice colors and 3 track colors)
'''
