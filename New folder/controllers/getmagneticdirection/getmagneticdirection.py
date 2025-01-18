from controller import Robot, Motor

# Create the Robot instance
robot = Robot()

# Time step of the simulation
TIME_STEP = 64

# Get the accelerometer device
acc = robot.getDevice('accelerometer')
acc.enable(TIME_STEP)

# Get the motor devices
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Set the motors to velocity control mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Define rotation parameters
rotation_speed = 2.0  # Speed of rotation
rotation_duration = 1200 / TIME_STEP  # Duration to rotate 90 degrees (adjust as needed)

while robot.step(TIME_STEP) != -1:
    # Start rotating the robot for 90 degrees
    left_motor.setVelocity(rotation_speed)
    right_motor.setVelocity(-rotation_speed)
    
    # Rotate for the calculated duration
    for _ in range(int(rotation_duration)):
        if robot.step(TIME_STEP) == -1:
            break
        
        # Read and print the accelerometer values
        acc_values = acc.getValues()
        print(f"Accelerometer values: x = {acc_values[0]:.2f}, y = {acc_values[1]:.2f}, z = {acc_values[2]:.2f}")
    
    # Stop the motors after completing the rotation
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
    # Wait for a moment before the next rotation
    for _ in range(100):  # Adjust the number of steps as needed
        if robot.step(TIME_STEP) == -1:
            break
    print("turn completed")
