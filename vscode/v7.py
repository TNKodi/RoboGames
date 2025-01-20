from controller import Robot,Motor,Camera,Accelerometer
robot = Robot()
import math 
robo_state=1
timestep = 64
max_speed=6.28

wkp=0.01 #0.01
wkd=0.046 #0.01
wki=0

temdis=913
count=0
ePrevious=0
eIntegral=0
init=False
cellsize=2.24
step=cellsize

detectcolour=0

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)

right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

left_ps = robot.getDevice('left wheel sensor')
left_ps.enable(timestep)

right_ps = robot.getDevice('right wheel sensor')
right_ps.enable(timestep)

cam=robot.getDevice('camera')
cam.enable(timestep)
cam.recognitionEnable(timestep)

acce = robot.getDevice('accelerometer')
acce.enable(timestep)

gyro = robot.getDevice('gyro')
gyro.enable(timestep)
ps_values=[0,0]
ps_values[0] = left_ps.getValue()
ps_values[1] = right_ps.getValue()
prox_sensors=[]
dis_values=[0]*8 

red_yellow=[+1,+1,-1,-1,+1,+1,-1,+1,+1,-1,2]
yellow_pink=[-1,-1,+1,-1,-1,+1,-1,-1,+1,+1,-1,-1,2]
pink_brown=[-1,-1,0,-1,2]
brown_green=[1,1,-1,1,1,1,-1,0,1,1,-1,1,2]

green_red=[-1,-1,0,1,-1,-1,1,1,-1,-1,-1,1,-1,0,-1,2]
pink_red=[-1,-1,2]
brown_red=[0,-1,2]
yellow_red=[+1,-1,-1,+1,-1,-1,+1,+1,-1,+1,2]

color_array=["red","yellow","pink","brown","green"]
color_path_array=[red_yellow,yellow_pink,pink_brown,brown_green]
color_path_index=0

red_x_array=[yellow_red,pink_red,brown_red,green_red]
red_curr_index=0
red_curr_path=red_x_array[0]

current_path=color_path_array[color_path_index]
curr_index=0
curr_index1=0

for ind in range(8):
    sensorname='ps'+str(ind)
    prox_sensors.append(robot.getDevice(sensorname))
    prox_sensors[ind].enable(timestep)
def forward():
    left_motor.setVelocity(0.6*max_speed)
    right_motor.setVelocity(0.6*max_speed)
def stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
def cellbreak():
    global init,step
    if(init==False):init=right_ps.getValue()
    else:
        pulses = (ps_values[1]-init)/ (2 * math.pi)
        print(pulses)
        if (pulses>=step):
            print('===============cellbreak===============',round(step//cellsize))
            step+=cellsize
def detect_color(rgb_values):
    r, g, b = rgb_values
    if 60 < r < 100 and g < 20 and b < 20:  # Red detection
        return 1
    if r > 200 and g > 200 and b < 40:  # Yello detection
        return 2
    elif r > 200 and g < 40 and b > 200:  # Pink detection
        return 3
    elif 100 < r < 160 and 90 < g < 120 and b < 60:  # Brown detection
        return 4
    elif r < 40 and g > 200 and b < 40:  # Green detection
        return 5
    else:
        return 0
def getColour():
    if dis_values[7]<=1900 and dis_values[0]>=1200:
        middle_color = detect_color(image[360][260])
        left_corner_color = detect_color(image[360][0])
        right_corner_color = detect_color(image[360][-1])
        if middle_color != 0:
            return middle_color
            #print(f"Middle pixel detected: {detectcolour}")
        elif left_corner_color !=0:
            return left_corner_color
            #print(f"Middle-left corner detected: {detectcolour}")
        elif right_corner_color != 0:
            return right_corner_color
        else :return 0           
def moveMotor(error):
    speed =abs(error)
    print("error ",error)
    # if (speed+0.7*max_speed > max_speed):
        # speed = 0.26*max_speed*error/speed
    # else:speed=error
    # left_motor.setVelocity(0.74*max_speed-speed)
    # right_motor.setVelocity(0.74*max_speed+speed)
    if (speed+0.8*max_speed >= max_speed):
        speed = 0.198*max_speed*error/speed
    else:speed=error
    left_motor.setVelocity(0.8*max_speed-speed)
    right_motor.setVelocity(0.8*max_speed+speed)

def pidController(kp,  kd, ki) : 
    global temdis,count
    if dir[2]==2:
        e=-(dis_values[2]-temdis)
        print("rrrrrrrr",temdis)
        count=0
    elif dir[1]==1:
        e=dis_values[5]-temdis
        print("LLLLLLLLLLLLLL",temdis)
        count=0
    elif dir[2]!=2 and dir[1]!=1:
        count+=1
        if count>5:
            e=dis_values[5]-dis_values[2]
            temdis=dis_values[2]
            
        else:e=0
    print('diff',e)
    global ePrevious
    global eIntegral
    eDerivative = (e - ePrevious)
    eIntegral = eIntegral + e
    u = (kp * e) + (kd * eDerivative) + (ki * eIntegral)
    ePrevious = e
    return u

def wallfallowPID():
    #forward()
    u = pidController(wkp, wkd, wki)
    moveMotor(u)

def isblock():
    dir=[10,10,10]
    frontl=[]
    frontr=[]
    left=[]
    right=[]
    for i in range(10):
        dis_values[0]=round(prox_sensors[0].getValue(),2)
        dis_values[7]=round(prox_sensors[7].getValue(),2)
        dis_values[5]=round(prox_sensors[5].getValue(),2)
        dis_values[2]=round(prox_sensors[2].getValue(),2)
           # print(ind,"-",dis_values[ind],end=' ')
        frontr.append(dis_values[0])
        frontl.append(dis_values[7])
        left.append(dis_values[5])
        right.append(dis_values[2])
    frontl.sort()
    frontr.sort()
    left.sort()
    right.sort() 
    print(left[5],frontl[5],frontr[5],right[5])
    if frontl[5]<1470 and frontr[5]<=1470:
        dir[0]= 0
    if right[5]>1980:
        dir[1]= 1
    if left[5]>1980:
        dir[2]= 2 
    return dir

def delay(seconds):
    global detectcolour
    steps = int(seconds * 1000 / timestep)  # Calculate the number of steps
    for _ in range(steps):
        dir=isblock()
        temcolour=getColour()
        if temcolour!=0 and not(temcolour is None):
            detectcolour=temcolour    
        robot.step(timestep)
        print('detect ',detectcolour)

def delayPID(seconds):
    steps = int(seconds * 1000 / timestep)  # Calculate the number of steps
    for _ in range(steps):
        wallfallowPID()
        dir=isblock()
        print("DDDDDDDDDDDDDDDDDDDD")
        robot.step(timestep)

def turn(dir):
    if dir==1:
        left_motor.setVelocity(0.3*max_speed)
        right_motor.setVelocity(-0.3*max_speed)
        delay(1.23)
    if dir==2:
        left_motor.setVelocity(-0.3*max_speed)
        right_motor.setVelocity(0.3*max_speed) 
        delay(1.23) 

def colortoindex():
    global red_curr_index
    global red_curr_path
    global detectcolour
    global robo_state
    if detectcolour==2:
        red_curr_index=0
        red_curr_path=red_x_array[0]
    elif detectcolour==3:
        red_curr_index=1
        red_curr_path=red_x_array[1]
    elif detectcolour==4:
        red_curr_index=2
        red_curr_path=red_x_array[2]
    elif detectcolour==5:
        red_curr_index=3
        red_curr_path=red_x_array[3]
    elif detectcolour==1:
        robo_state=3

def findColour():
    global robo_state

    image = cam.getImageArray()
    dir=isblock()
    wallfallowPID()
    print(dir)  
    print('detect ',detectcolour)
    if dir[0]==0:
        if dir[1]==1:
            init=False
            step=cellsize
            turn(1)
        elif dir[2]==2:
            turn(2)
            init=False
            step=cellsize
        else:
            turn(1)
            turn(1)
            init=False
            step=cellsize    
    if dir[0]==10:
        if dir[2]==2:
            forward()
            delay(1.44)
            turn(2)
            forward()
            delay(1.9)
            # forward()
            # delay(1.9)
            # print("pppppppppppppppppppppp")
            # delayPID(1.1)
            # turn(2)
            # forward()
            # delay(1.1)
            # print("====================")
            #delayPID(1.5)
    if detectcolour:
        stop()
        delay(20)
        robo_state=2
        colortoindex()

#--------------------------------------------------------------------------------
def pidController1(kp,  kd, ki,dir) : 
    global temdis,count
    if dir[2]==2:
        e=-(dis_values[2]-temdis)
        print("rrrrrrrr",temdis)
        count=0
    elif dir[1]==1:
        e=dis_values[5]-temdis
        print("LLLLLLLLLLLLLL",temdis)
        count=0
    elif dir[2]!=2 and dir[1]!=1:
        count+=1
        if count>5:
            e=dis_values[5]-dis_values[2]
            temdis=dis_values[2]
            
        else:e=0
    print('diff',e)
    global ePrevious
    global eIntegral
    eDerivative = (e - ePrevious)
    eIntegral = eIntegral + e
    u = (kp * e) + (kd * eDerivative) + (ki * eIntegral)
    ePrevious = e
    return u

def isblock1():
    dir=[10,10,10]
    left=[]
    right=[]
    for i in range(10):
        for ind in range(8):
            dis_values[ind]=round(prox_sensors[ind].getValue(),2)
           # print(ind,"-",dis_values[ind],end=' ')
        left.append(dis_values[5])
        right.append(dis_values[2])
    left.sort()
    right.sort() 
    print(left[5],right[5])
    if right[5]>1980:
        dir[1]= 1
    if left[5]>1980:
        dir[2]= 2 
    return dir

def wallfallowPID1():
    dir=isblock1()
    u = pidController1(wkp, wkd, wki,dir)
    moveMotor(u)


def isWalldetect():
    frontl=[]
    frontr=[]
    for i in range(10):
        frontr.append(round(prox_sensors[0].getValue(),2))
        frontl.append(round(prox_sensors[7].getValue(),2))
    frontl.sort()
    frontr.sort()
    if(frontl[5]<1570 and frontr[5]<1570):
        stop()
        return False
    return True

def delay1(seconds):
    steps = int(seconds * 1000 / timestep)  # Calculate the number of steps
    for _ in range(steps):
        robot.step(timestep)

def part3():
    global curr_index
    global current_path
    global color_path_index
    print(curr_index,color_path_index)
    if(current_path[curr_index]==0):
        print(22222)
        print(color_path_index,curr_index,1111111111111111111111111)
        if (color_path_index==2 and curr_index==2):
            forward()
            delay1(3.4)
            stop()
            turn(1)
            stop()
            forward()
            delay1(2.4)
            stop()
            curr_index+=1
        if (color_path_index==3 and curr_index==7):
            forward()
            delay1(2.8)
            stop()
            delay1(2)
            print(222222222222222222222222222222222222222222222222222222)
            turn(1)
            stop()
            forward()
            delay1(2.7)
            stop()
            curr_index+=1
    elif isWalldetect():
        wallfallowPID1()
    else:
        if(len(current_path)-1==curr_index):
            stop()
            delay1(2)
            print("pathchanged",11111111111111111111111111111111111111111111111)
            color_path_index+=1
            current_path=color_path_array[color_path_index]
            curr_index=0
        elif (current_path[curr_index]==1):
            turn(2)
            stop()
            curr_index+=1
        elif(current_path[curr_index]==-1):
            turn(1)
            stop()
            curr_index+=1




def part2():
    global curr_index1
    global red_curr_path
    global red_curr_index
    global robo_state
    current1_path=red_curr_path
    color1_path_index=red_curr_index
    if(current1_path[curr_index1]==0):
        print(22222)
        print(color1_path_index,curr_index1,1111111111111111111111111)
        if (color1_path_index==3 and curr_index1==2):
            forward()
            delay1(3.4)
            stop()
            turn(2)
            stop()
            forward()
            delay1(2.4)
            stop()
            curr_index1+=1
        if (color1_path_index==3 and curr_index1==13):
            forward()
            delay1(3.14)
            stop()
            curr_index1+=1
        if (color1_path_index==2 and curr_index1==0):
            forward()
            delay1(3.4)
            stop()
            curr_index1+=1
    elif isWalldetect():
        wallfallowPID1()
    else:
        if(len(current1_path)-1==curr_index1):
            stop()
            delay1(2)
            curr_index1==0
            robo_state=3
        elif (current1_path[curr_index1]==1):
            turn(2)
            stop()
            curr_index1+=1
        elif(current1_path[curr_index1]==-1):
            turn(1)
            stop()
            curr_index1+=1

image = cam.getImageArray()
for i in range(720):
    a=detect_color(image[360][i])
    

while robot.step(timestep) != -1:
    

    ps_values[0] = left_ps.getValue()
    ps_values[1] = right_ps.getValue()
    cellbreak()
    image = cam.getImageArray()
    
    if robo_state==3:
        print(111)
        part3()
    elif robo_state==2:
        print(1010101001001)
        part2()
    elif robo_state==1:
        dir=isblock()
        findColour()