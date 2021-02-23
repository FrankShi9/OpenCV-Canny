import cv2 #openCV
import math 
import numpy as np #data frame

#drone control and simulation import
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

#print the full content of the value of a numpy object
np.set_printoptions(threshold=np.inf)

#openCV video live capture
cap = cv2.VideoCapture(0)
 
printCnt = True

#used in detect()
coefficient = 0.02

#---------------------------------------------------------------------------------------------------

async def takeoff():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break
    print("-- Arming")
    await drone.action.arm()
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(10)

#---------------------------------------------------------------------------------------------------

async def takeoff1():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Go 0m North, 0m East, -5m Down within local coordinate system")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
    await asyncio.sleep(10)
#---------------------------------------------------------------------------------------------------
async def land():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone...")
    await asyncio.sleep(20)
    async for state in drone.core.connection_state():
       if state.is_connected:
           print(f"Drone discovered with UUID: {state.uuid}")
           break
    print("-- Landing")
    await drone.action.land()

#---------------------------------------------------------------------------------------------------

#other methods to achieve this 
async def adjust():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    
    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        return

    print("-- Adjusting")
    await drone.offboard.set_position_ned(PositionNedYaw(5.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(10)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")
    

class ShapeDetector:
    
    def __init__(self):
        #dictionary counter stores the total count of all shapes detected
        self.counter = {'unrecognized image': 0, 'triangle': 0, 'general rectangle': 0, 'right rectangle': 0, 'pentagon': 0, 'hexagon': 0}
        #initialize shape attribute as 'unrecognized image'
        self.shape = 'unrecognized image'
        #initialize an empty list to the vertices list
        self.vertices = []
        #initial the perimeter to 0
        self.peri = 0
#---------------------------------------------------------------------------------------------------
    #calculate the Euclidean distance (mainly used to distinguish rhombuses and rectangles since a rhombus would have sides with a equal length)
    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

#---------------------------------------------------------------------------------------------------

    def detect(self, c):
        #cv2.arcLength function returns perimeter
        self.peri = cv2.arcLength(c, True)
        #cv2.approxPolyDP function uses polygons to fit the input data and returns a list of vertices
        self.vertices = cv2.approxPolyDP(c, coefficient*self.peri, True)


        if len(self.vertices) == 3: #顶点个数
            self.shape = "triangle"


        elif len(self.vertices) == 4:

            dist1 = self.distance(self.vertices[0][0][0], self.vertices[0][0][1], self.vertices[2][0][0],self.vertices[2][0][1]) #对角顶点1
            dist2 = self.distance(self.vertices[1][0][0], self.vertices[1][0][1], self.vertices[3][0][0],self.vertices[3][0][1]) #对角顶点2

            result = math.fabs(dist1 - dist2) #两对角线差

            if result <= 10: #两对角线几乎相等
                self.shape = "right rectangle"
            else:
                self.shape = "general rectangle"

        elif len(self.vertices) == 5:
            self.shape = "pentagon"

        elif len(self.vertices) == 6:
            self.shape = "hexagon"



        self.counter[self.shape] += 1

        return self.shape

#---------------------------------------------------------------------------------------------------


    def display(self):

        for type in self.counter.keys():
            print("The number of {} is {}".format(type, self.counter[type]))

#---------------------------------------------------------------------------------------------------




async def run():

    global asyncio, System
    loop = asyncio.get_event_loop()

    #take off
    #loop.run_until_complete(takeoff1()) 
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Go 0m North, 0m East, -5m Down within local coordinate system")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
    await asyncio.sleep(15)


    #real time video cam edge canny 
    global cv2, printCnt, np
    while True:
        _,frame = cap.read()
        # if frame.isEmpty():
        #break
        frame=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        frame=cv2.blur(frame, (7,7))
        frame=cv2.Canny(frame,0,30,3)
        cv2.imshow("real time edge detect:",frame)

        counts = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #只需要取轮廓上点的信息

        counts = counts[1]

        #创建一个识别器实例
        test_sd = ShapeDetector()

        #分别对每个轮廓进行处理
        for c in counts:

            shape = test_sd.detect(c)

        #test_sd.display()

        #data collection of 0s and 255s in a frame (numpy)
        
        #if printCnt == True:
            #print(frame)
            #np.savetxt("data.txt", frame)
            #printCnt = False
        
        

        print('**'+str(test_sd.counter['right rectangle'])+'**')


        
        #if test_sd.counter['right rectangle'] >= 20: #while safe land condition is not met
        if test_sd.counter['right rectangle'] == 0:    
            #loop.run_until_complete(adjust())
            print("-- Adjusting")
            await asyncio.sleep(10)
            await drone.offboard.set_position_ned(PositionNedYaw(5.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(10)

            print("-- Stopping offboard")
            try:
                await drone.offboard.stop()
            except OffboardError as error:
                print(f"Stopping offboard mode failed with error code: {error._result.result}")
        

        #land
        #loop.run_until_complete(land()) 
        print("-- Landing")
        await asyncio.sleep(5)
        await drone.action.land()
        break
        a=cv2.waitKey(30)
        if a == 27:  #exit real time cam canny
            break

    cap.release() #release camera live capture
    del cv2
    del np

    
    

    #program entrance
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())