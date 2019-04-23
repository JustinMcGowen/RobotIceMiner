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
            return 'there'
        else:
            drive_time=dist/velocity

            DRIVE=800
            if drive_time<0: #if time ends up being negative, make it positive, but turn the other direction
                drive_time=0-drive_time
                DRIVE=0-DRIVE
            if not self.timer.isAlive(): #dont send more wait commands to motors if they are already moving
                #self.timer.join()
                print('starting thread')
                self.timer = Timer(drive_time)
                self.timer.start()
                self.motors=6000-DRIVE
                self.motorValues=[self.motors, self.turn, self.body, self.headTilt, self.headTurn, self.arm, self.hand]
                self.tango.setTarget(self.motorList[0], self.motorValues[0])
            return 'not there'

    def rotate(self,angle,desired,velocity):
        if desired-angle<5 and desired-angle>-5: #if we are within 5 degrees of our target, stop motors
            self.killMotors()
            return 'there'
        else:
            rotate_time = (desired-angle)/velocity
            TURN=800
            if rotate_time<0: #if time ends up being negative, make it positive, but turn the other direction
                rotate_time=0-rotate_time
                TURN=0-TURN
            if not self.timer.isAlive(): #dont send more wait commands to motors if they are already moving
                self.lastCommandTime=time.time()
                #timer.join()
                self.timer = Timer(rotate_time)
                timer.start()
                self.turn=6000+TURN
                self.motorValues=[self.motors, self.turn, self.body, self.headTilt, self.headTurn, self.arm, self.hand]
                self.tango.setTarget(self.motorList[1], self.motorValues[1])
            return 'not there'

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

    def tiltHead(self, pos):
        if pos == 'low':
            self.headTilt = 4000  #maybe 8000?
        if pos == 'mid':
            self.headTilt = 6000
        if pos == 'high':
            self.headTilt = 7400 #0r 4600
        self.motorValues=[self.motors, self.turn, self.body, self.headTilt, self.headTurn, self.arm, self.hand]
        self.tango.setTarget(self.motorList[3], self.motorValues[3])

    def moveArm(self, action):
        if action == 'extend':
            self.arm = 7900
        if action == 'retract':
            self.arm = 4000
        if action == 'close':
            self.hand = 7400
        if action == 'open':
            self.hand = 5000
        self.motorValues=[self.motors, self.turn, self.body, self.headTilt, self.headTurn, self.arm, self.hand]
        self.tango.setTarget(self.motorList[5], self.motorValues[5])
        self.tango.setTarget(self.motorList[6], self.motorValues[6])

class WhereAmI():
    def __init__(self, phone):
        self.phone=phone
        self.angle = -1
        self.x = -1
        self.y = -1
        self.fps = 0.8  #feet per second (.8fps at default)
        self.dps = -1  #degrees per second
        self.hasIce = False
        self.tasks = ['find human']#['probe position','move to start','probe speed','enter obstacle stage','find human','verify color','traverse obstacles','drop payload']
        self.location = 'start' #can be start, intermediate, end
        self.commandExcecuted=False
        self.found=False
        self.timeStart = time.time()

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
        if not hasIce:
            self.phone.sendData("Now entering the mining area")
        else:
            pass
    def findHuman(self,frame):
        face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        #cv2.imshow("findHuman",frame)

        for (x,y,w,h) in faces:
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0))

        self.bot.tiltHead('high')

        if self.angle>=170:
            self.bot.rotate(self.angle,-180,self.dps)
        else:
            self.bot.rotate(self.angle,180,self.dps)
        #self.bot.rotate(self.angle,)

        for (x,y,w,h) in faces:
            print("("+str(x)+","+str(y)+") w="+str(w)+" h="+str(h))
            if w >= 89 and w <= 120:
                self.tasks=self.tasks[1:]
                self.phone.sendData("I crave that pink ice")


    def detectBall(self,frame):
        #Human should have already been found
        self.bot.moveArm("open")
        self.bot.moveArm("extend")
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
        print(len(conts))
        if len(conts) == 1:
            count +=1
            print(count)
            self.found = True
            for i in range(len(conts)):
                x,y,w,h=cv2.boundingRect(conts[i])
        else:
            self.found = False

        if self.found == True and count == 75:
            self.bot.moveArm("close")
            self.bot.moveArm("retract")
            self.tasks=self.tasks[1:]
            self.phone.sendData("Thank you for your service")

        else if foundColor == False:
            count = 0

        else:
            pass
    def dropPayload(self,frame):
        pass


    def testSpeed(self):
        print('start test')
        '''self.bot.move(2,self.fps)
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

    IP = '10.200.3.102'
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
