from controller import Robot,Motor,Camera,Accelerometer
robot = Robot()
import math 
timestep = 64
max_speed=6.28
wkp=0.01 #0.01
wkd=0.046 #0.01
wki=0
temdis=0
count=0
ePrevious=0
eIntegral=0
init=False
cellsize=2.24
step=cellsize

# enble Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)

right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)
#motor Encodes
left_ps = robot.getDevice('left wheel sensor')
left_ps.enable(timestep)

right_ps = robot.getDevice('right wheel sensor')
right_ps.enable(timestep)
#cam Access
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
prox_sensors=[] #get distnace sensor array
dis_values=[0]*8   # [0] /[7] infront 75
#[2] right 86   
# 
#                [5] left  86


#----------------------------------------------------------------------------------------
red_yellow=[1,+1,-1,-1,+1,+1,-1,+1,+1,-1,2]
yellow_pink=[-1,-1,+1,-1,-1,+1,-1,-1,+1,+1,-1,-1,2]
pink_brown=[-1,-1,0,-1,2]
brown_green=[1,1,-1,1,1,1,-1,0,1,1,-1,1,2]
green_red=[-1,-1,-1,-1,0,1,-1,-1,1,1,-1,-1,-1,1,-1,0,-1,2]
pink_red=[-1,-1,2]
brown_red=[-1,0,-1,2]
yellow_red=[-1,-1,+1,-1,-1,+1,-1,-1,+1,+1,-1,+1,2]



color_array=["red","yellow","pink","brown","green"]
color_path_array=[red_yellow,yellow_pink,pink_brown,brown_green,green_red,pink_red,brown_red,yellow_red]
color_path_index=7

current_path=color_path_array[color_path_index]
curr_index=0
#-------------------------------------------------------------------------------------

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

def pidController(kp,  kd, ki,dir) : 
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

def isblock():
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

def wallfallowPID():
    dir=isblock()
    u = pidController(wkp, wkd, wki,dir)
    moveMotor(u)




def delay(seconds):
    steps = int(seconds * 1000 / timestep)  # Calculate the number of steps
    for _ in range(steps):
        robot.step(timestep)

def turn(dir):
    if dir==1:
        left_motor.setVelocity(0.3*max_speed)
        right_motor.setVelocity(-0.3*max_speed)
        delay(1.174)
        
    if dir==2:
        left_motor.setVelocity(-0.3*max_speed)
        right_motor.setVelocity(0.3*max_speed) 
        delay(1.174) 

#---------------------------------------------------------------------------

def isWalldetect():
    frontl=[]
    frontr=[]
    for i in range(10):
        frontr.append(round(prox_sensors[0].getValue(),2))
        frontl.append(round(prox_sensors[7].getValue(),2))
    frontl.sort()
    frontr.sort()
    if(frontl[5]<1470 and frontr[5]<1470):
        stop()
        return False
    return True
    
def colourDetect():
    color="RED"
    return color
#----------------------------------------------------------------------

print(right_ps.getValue())

while robot.step(timestep) != -1:
    

    ps_values[0] = left_ps.getValue()
    ps_values[1] = right_ps.getValue()
    cellbreak()
    image = cam.getImageArray()
    

    print(111)
    stop()
    if(current_path[curr_index]==0):
        print(22222)
        print(color_path_index,curr_index,1111111111111111111111111)
        if (color_path_index==2 and curr_index==2):
            forward()
            delay(3.4)
            stop()
            turn(1)
            stop()
            forward()
            delay(2.4)
            stop()
            curr_index+=1
        if (color_path_index==3 and curr_index==7):
            forward()
            delay(2.8)
            stop()
            delay(2)
            print(222222222222222222222222222222222222222222222222222222)
            turn(1)
            stop()
            forward()
            delay(2.7)
            stop()
            curr_index+=1
        if (color_path_index==4 and curr_index==4):
            forward()
            delay(3.4)
            stop()
            turn(2)
            stop()
            forward()
            delay(2.4)
            stop()
            curr_index+=1
        if (color_path_index==4 and curr_index==15):
            forward()
            delay(3.14)
            stop()
            curr_index+=1
        if (color_path_index==6 and curr_index==1):
            forward()
            delay(3.14)
            stop()
            curr_index+=1
    elif isWalldetect():
        wallfallowPID()
    else:
        if(len(current_path)-1==curr_index):
            stop()
            delay(2)
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

    