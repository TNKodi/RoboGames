from controller import Robot,Motor,Camera
robot = Robot()

timestep = 64
max_speed=6.28
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

ps_values=[0,0]
ps_values[0] = left_ps.getValue()
ps_values[1] = right_ps.getValue()
prox_sensors=[]
dis_values=[0]*8   # [0] /[7] infront 75
#[2] right 86                  [5] left  86

for ind in range(8):
    sensorname='ps'+str(ind)
    prox_sensors.append(robot.getDevice(sensorname))
    prox_sensors[ind].enable(timestep)
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
    if frontl[5]+frontr[5]>144:
        dir[0]= 0
    if right[5]>69:
        dir[1]= 1
    if left[5]>69:
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
        delay(1.18)
    if dir==2:
        left_motor.setVelocity(-0.3*max_speed)
        right_motor.setVelocity(0.3*max_speed) 
        delay(1.18)    
while robot.step(timestep) != -1:
    #recognized_object_array = cam.getRecognitionObjects()
    ps_values[0] = left_ps.getValue()
    ps_values[1] = right_ps.getValue()
    print(ps_values)
    image = cam.getImageArray()
    print()
    print(image[200][300])
    left_motor.setVelocity(0.6*max_speed)
    right_motor.setVelocity(0.6*max_speed)
    dir=isblock()
    print(dir)
    if dir[0]==0:
        if dir[1]==1:
            turn(2)
        elif dir[2]==2:
            turn(1)