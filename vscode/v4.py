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
#[2] right 86                  [5] left  86

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
        for ind in range(8):
            dis_values[ind]=round(prox_sensors[ind].getValue(),2)
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
    if frontl[5]<1100 and frontr[5]<=1100:
        dir[0]= 0
    if right[5]>1980:
        dir[1]= 1
    if left[5]>1980:
        dir[2]= 2 
    return dir

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

print(right_ps.getValue())

while robot.step(timestep) != -1:
    #recognized_object_array = cam.getRecognitionObjects()
    ps_values[0] = left_ps.getValue()
    ps_values[1] = right_ps.getValue()
    cellbreak()
    # gy=gyro.getValues()
    # print('gy',gy)
    # s=acce.getValues()
    
    # print("Sh",s)
    # delay(2)
    image = cam.getImageArray()
    #print(image[200][300])
    dir=isblock()
    print(dir)
    wallfallowPID()
    # if dir[1]!=1 and dir[2]!=2:
    #     wallfallowPID()
    #     #forward()
    # else:
    #     wallfallowPID()
    #     # forward()
    #     # delay(.1)
            
    # if dir[0]==0:
    #     if dir[1]==1:
    #         print('rrrrrrrrrrrrrrrrrrrrrrrrr')
    #         turn(1)
    #     elif dir[2]==2:
    #         print('llllllllllllllllllllllllllllll')
    #         turn(2)
    #     else:
    #         print('========================')
    #         turn(1)
    #         turn(1)
    #         init=False
    #         step=cellsize
    #         print('pppppppppppppppppppppppppp')
        

