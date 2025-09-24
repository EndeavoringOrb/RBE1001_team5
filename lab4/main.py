# region VEXcode Generated Robot Configuration
from vex import *

# Brain should be defined by default
brain = Brain()

# Robot configuration code
left_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, not False)
right_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, not True)
arm_motor = Motor(Ports.PORT6, GearSetting.RATIO_18_1, not True)
# AI Vision Color Descriptions
# ai_vision_11__COLOR1 = Colordesc(1, 100, 255, 162, 8, 0.41)  # green
# ai_vision_11__COLOR2 = Colordesc(2, 181, 126, 220, 10, 0.43)  # purple
# ai_vision_11__COLOR3 = Colordesc(3, 227, 62, 59, 12, 0.14)  # orange
ai_vision_11__COLOR1 = Colordesc(1, 96, 102, 200, 10, 0.2)  # purple
ai_vision_11__COLOR2 = Colordesc(2, 20, 213, 105, 10, 0.2)  # green
ai_vision_11__COLOR3 = Colordesc(3, 206, 115, 102, 10, 0.2)  # orange
# AI Vision Code Descriptions
ai_vision_15 = AiVision(Ports.PORT11, ai_vision_11__COLOR2)
bumper_g = Bumper(brain.three_wire_port.g)
controller = Controller(PRIMARY)

# Add sonar and IMU from Lab 2
brain_inertial = Inertial(Ports.PORT3)
range_finder_e = Sonar(brain.three_wire_port.d)

BASE_SPEED = 85
CAM_WIDTH = 320
CAM_HEIGHT = 240
AREA_THRESHOLD = 0.7
ARM_AREA_THRESHOLD = 0.4
KNOCK_TIME = 1.3
knock_timer = Timer()
knock_turn_time = knock_timer.time() - 10
arm_knock_turn_time = knock_timer.time() - 10

# Constants from Lab 2
forwardkd = 10
baseSpeed = 200

ROBOT_IDLE = 0
ROBOT_SEARCHING = 1
ROBOT_APPROACHING = 2
ROBOT_TURN = 3  # Add turn state

current_state = ROBOT_IDLE

# Variables for wall detection
wasUlt = False
last_turn_time = 0
TURN_COOLDOWN = 1000  # 1 second in milliseconds

# wait for rotation sensor to fully initialize
wait(30, MSEC)

# endregion VEXcode Generated Robot Configuration

# ------------------------------------------
#
#   Project:      Lab 4 with Wall Avoidance
#   Author:       Modified from VEX
#   Created:
#   Description:  Lab 4 with Lab 2 wall avoidance integrated
#
# ------------------------------------------

"""
This code demonstrates a basic search and drive towards behaviour with the camera,
now with wall avoidance during the searching state.

The robot has four states:
    IDLE - waiting for the button press
    SEARCHING - moves forward with wall avoidance until it finds an object
    APPROACHING - drives towards the object
    TURN - performs 90-degree turn when wall detected
"""

# Timer for IMU-based forward movement
imuTimer = Timer()

"""
We'll use a timer to read the camera every cameraInterval milliseconds
"""
cameraInterval = 50
cameraTimer = Timer()


def handleButton():
    global current_state

    if current_state == ROBOT_IDLE:
        print("IDLE -> SEARCHING")
        current_state = ROBOT_SEARCHING
        # Start forward movement with IMU control
        imuTimer.event(handleForward, 100)
        # Start the timer for the camera
        cameraTimer.event(cameraTimerCallback, cameraInterval)

    else:  # failsafe; go to IDLE from any other state when button is pressed
        print(" -> IDLE")
        current_state = ROBOT_IDLE
        left_motor.stop()
        right_motor.stop()
        imuTimer.clear()  # Clear IMU timer

handleButton()
# controller.buttonL1.pressed(handleButton)

"""
Wall detection function from Lab 2
"""


def checkUltrasonic():
    global wasUlt

    ult = range_finder_e.distance(MM) > 40 and range_finder_e.distance(MM) < 400

    if ult:
        print("Wall detected at {} mm".format(range_finder_e.distance(MM)))

    retVal = False
    if not wasUlt and ult:
        retVal = True
    wasUlt = ult
    return retVal


"""
Forward movement with IMU control from Lab 2
"""


def handleForward():
    if current_state != ROBOT_SEARCHING:
        return

    angle = brain_inertial.rotation()
    left_motor.spin(FORWARD, baseSpeed - (forwardkd * angle))
    right_motor.spin(FORWARD, baseSpeed + (forwardkd * angle))
    imuTimer.event(handleForward, 100)


"""
We'll keep track of missed detections. If it exceeds some threshold, go back to SEARCHING
"""
missedDetections = 0


def checkForLostObject():
    if missedDetections > 20:
        return True
    else:
        return False


def handleLostObject():
    global current_state
    if current_state == ROBOT_APPROACHING:
        print("APPROACHING -> SEARCHING")
        current_state = ROBOT_SEARCHING
        # Resume forward movement with wall avoidance
        imuTimer.event(handleForward, 100)


def cameraTimerCallback():
    global current_state
    global missedDetections

    objects = ai_vision_15.take_snapshot(ai_vision_11__COLOR2)
    if objects:
        handleObjectDetection()
    else:
        arm_motor.stop()
        print("no objects detected", end="\r")
        missedDetections = missedDetections + 1

    # restart the timer
    if current_state != ROBOT_IDLE:
        cameraTimer.event(cameraTimerCallback, cameraInterval)


def handleObjectDetection():
    global current_state, missedDetections, knock_turn_time, arm_knock_turn_time

    obj = ai_vision_15.largest_object()
    area_pct = (obj.height * obj.width) / (CAM_HEIGHT * CAM_WIDTH)
    cy = obj.centerY
    cx = obj.centerX
    brain.screen.print_at("area:", area_pct, x=100, y=100)
    brain.screen.print_at("height:", obj.height, x=100, y=120)
    brain.screen.print_at("width:", obj.width, x=100, y=140)
    brain.screen.print_at("cy:", cy, x=100, y=160)

    if current_state == ROBOT_SEARCHING:
        print("SEARCHING -> APPROACHING")
        current_state = ROBOT_APPROACHING
        imuTimer.clear()  # Stop the forward movement timer

    # Not elif, because we want the logic to cascade
    if current_state == ROBOT_APPROACHING:
        # horizontal
        target_y = CAM_HEIGHT / 2
        K_y = 0.5

        error = cy - target_y
        if area_pct > AREA_THRESHOLD:
            knock_turn_time = knock_timer.time()
        knock_check = knock_timer.time() - knock_turn_time < KNOCK_TIME
        if knock_check:
            error += 600
        turn_effort = K_y * error
        brain.screen.print_at("turn_effort:", turn_effort, x=100, y=180)

        left_motor.spin(FORWARD, BASE_SPEED + turn_effort)
        right_motor.spin(FORWARD, BASE_SPEED - turn_effort)

        # vertical
        target_x = 0.25 * CAM_WIDTH
        K_x = 1.0

        error = cx - target_x
        if area_pct > ARM_AREA_THRESHOLD:
            arm_knock_turn_time = knock_timer.time()
        arm_knock_check = knock_timer.time() - arm_knock_turn_time < KNOCK_TIME
        if arm_knock_check or knock_check:
            error = 0
        turn_effort = K_x * error
        arm_motor.spin(FORWARD, turn_effort)

        if knock_check:
            sleep(int(KNOCK_TIME * 1000), TimeUnits.MSEC)

    # reset the time out timer
    missedDetections = 0


def handle90DegreeTurn():
    """Perform a 90-degree turn when wall is detected"""
    global current_state, last_turn_time

    if current_state == ROBOT_SEARCHING:
        print("Performing 90-degree turn")

        # Clear the IMU timer to stop forward movement
        imuTimer.clear()

        # Stop motors
        left_motor.stop()
        right_motor.stop()

        # Perform the actual turn - spin in place
        left_motor.spin(FORWARD, 150, RPM)
        right_motor.spin(REVERSE, 150, RPM)

        # Wait for turn to complete (adjust time as needed for 90 degrees)
        wait(700, MSEC)

        # Stop turning
        left_motor.stop()
        right_motor.stop()

        # Update the IMU heading by 90 degrees
        brain_inertial.set_rotation(brain_inertial.rotation() + 90)
        last_turn_time = brain.timer.time(MSEC)

        # Small pause before resuming
        wait(200, MSEC)

        # Resume forward movement
        imuTimer.event(handleForward, 100)
        print("Turn complete, resuming search")


# Main loop
while True:
    # Display current state and sensor info
    brain.screen.print_at("State: {}".format(current_state), x=10, y=20)
    brain.screen.print_at(
        "Sonar: {} mm".format(range_finder_e.distance(MM)), x=10, y=40
    )

#     # Check for wall detection only during searching state
#     if current_state == ROBOT_SEARCHING:
#         if checkUltrasonic():
#             current_time = brain.timer.time(MSEC)
#             # Only turn if enough time has passed since last turn
#             if current_time - last_turn_time >= TURN_COOLDOWN:
#                 handle90DegreeTurn()

#     # Check if object is lost during approaching
#     if checkForLostObject():
#         handleLostObject()

#     wait(20, MSEC)  # Small delay to prevent CPU overload
