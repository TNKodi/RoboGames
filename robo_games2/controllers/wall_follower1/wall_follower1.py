from controller import Robot, DistanceSensor, Motor

# Time step of the simulation
TIME_STEP = 64

# PID constants
Kp = 0.5
Ki = 0.0
Kd = 0.0

# Desired distance from the wall (in meters)
desired_distance = 0.001

# Initialize the robot
robot = Robot()

# Initialize distance sensors
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

# Initialize motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# PID variables
integral = 0.0
previous_error = 0.0

while robot.step(TIME_STEP) != -1:
    # Read distance sensor values
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # Calculate the error (difference between desired and actual distance)
    actual_distance = psValues[0]  # Assuming ps0 is the front sensor
    error = desired_distance - actual_distance

    # Calculate PID terms
    integral += error * TIME_STEP
    derivative = (error - previous_error) / TIME_STEP
    previous_error = error

    # Calculate control signal
    control_signal = Kp * error + Ki * integral + Kd * derivative

    # Set motor velocities
    leftMotor.setVelocity(3 - control_signal)
    rightMotor.setVelocity(3 + control_signal)

    # Print the control signal for debugging
    print(f"Control Signal: {control_signal}")

# Cleanup
robot.cleanup()
