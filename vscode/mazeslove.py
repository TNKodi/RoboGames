from controller import Robot

# Create the Robot instance
robot = Robot()

# Time step of the simulation
TIME_STEP = 64

# Main loop
while robot.step(TIME_STEP) != -1:
    # Example: Print "Hello, Webots!" in each step
    print("Hello, Webots!")

# Cleanup code if needed
