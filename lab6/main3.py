# region VEXcode Generated Robot Configuration
from vex import *

# Brain should be defined by default
brain = Brain()

# Robot configuration code - Port Bindings
left_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, not False)
right_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, not True)
arm_motor = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)
clamp_motor = Motor(Ports.PORT19, GearSetting.RATIO_18_1, True)
ultrasonic_sensor = Sonar(brain.three_wire_port.f)
arm_motor.set_stopping(BRAKE)

# AI Vision Color Descriptions - All three colors configured
ai_vision_11__COLOR1 = Colordesc(1, 100, 255, 162, 8, 0.41)  # green
ai_vision_11__COLOR2 = Colordesc(2, 181, 126, 220, 10, 0.43)  # purple
ai_vision_11__COLOR3 = Colordesc(3, 227, 62, 59, 12, 0.14)  # orange

# AI Vision Configuration with all three colors
ai_vision_15 = AiVision(
    Ports.PORT4, ai_vision_11__COLOR1, ai_vision_11__COLOR2, ai_vision_11__COLOR3
)

# Additional sensors
bumper_g = Bumper(brain.three_wire_port.g)
controller = Controller(PRIMARY)
brain_inertial = Inertial(Ports.PORT3)
range_finder_e = Sonar(brain.three_wire_port.d)

# Configuration constants
BASE_SPEED = 85
CAM_WIDTH = 320
CAM_HEIGHT = 240
AREA_THRESHOLD = 0.4
ARM_AREA_THRESHOLD = 0.4
KNOCK_TIME = 1.5
knock_timer = Timer()
knock_turn_time = knock_timer.time() - 10
arm_knock_turn_time = knock_timer.time() - 10

# Arm position constants
ARM_SEARCH_POSITION = 230  # Position when searching
ARM_SPEED = 50  # Speed for moving to search position
ARM_MAX_POSITION = 800  # Maximum arm position before returning to search
ARM_MIN_POSITION = 100  # Minimum arm position before returning to search

# Constants from Lab 2
forwardkd = 10
baseSpeed = 200

# Robot states
ROBOT_IDLE = 0
ROBOT_SEARCHING = 1
ROBOT_APPROACHING = 2
ROBOT_TURN = 3
ROBOT_PICKUP_BOX = 4


current_state = ROBOT_IDLE

# Variables for wall detection
wasUlt = False
last_turn_time = 0
TURN_COOLDOWN = 1000  # 1 second in milliseconds

# Color tracking variables
target_color = None  # Will store which color we're currently tracking
color_priority = [
    ai_vision_11__COLOR1,
    ai_vision_11__COLOR2,
    ai_vision_11__COLOR3,
]  # Priority order
color_names = {1: "GREEN", 2: "PURPLE", 3: "ORANGE"}  # For display purposes

# wait for rotation sensor to fully initialize
wait(30, MSEC)

# endregion VEXcode Generated Robot Configuration

# ------------------------------------------
#
#   Project:      Lab 4 with Wall Avoidance and Multi-Color Detection
#   Author:       Modified from VEX
#   Created:
#   Description:  Lab 4 with Lab 2 wall avoidance and all 3 color detection
#
# ------------------------------------------

"""
This code demonstrates a basic search and drive towards behaviour with the camera,
now with wall avoidance during the searching state and detection of all 3 colors.

The robot has four states:
    IDLE - waiting for the button press
    SEARCHING - moves forward with wall avoidance until it finds any colored object (arm at -230)
    APPROACHING - drives towards the detected colored object
    TURN - performs 90-degree turn when wall detected
"""

# Timer for IMU-based forward movement
imuTimer = Timer()

"""
We'll use a timer to read the camera every cameraInterval milliseconds
"""
cameraInterval = 50
cameraTimer = Timer()


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
Function to move arm to search position
"""


def moveArmToSearchPosition():
    arm_motor.spin_to_position(ARM_SEARCH_POSITION, DEGREES, ARM_SPEED, RPM, False)


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
    global current_state, target_color
    if current_state == ROBOT_APPROACHING:
        print(
            "APPROACHING -> SEARCHING (lost {})".format(
                color_names.get(target_color.id if target_color else 0, "object")
            )
        )
        current_state = ROBOT_SEARCHING
        target_color = None  # Clear the target color
        # Move arm back to search position
        moveArmToSearchPosition()
        # Resume forward movement with wall avoidance
        imuTimer.event(handleForward, 100)


def detectAllColors():
    """
    Check for all three colors and return the largest detected object
    along with which color it is
    """
    largest_obj = None
    detected_color = None
    max_area = 0

    # Check each color in priority order
    for color in color_priority:
        objects = ai_vision_15.take_snapshot(color)
        if objects:
            obj = ai_vision_15.largest_object()
            if obj:
                area = obj.height * obj.width
                # Keep track of the largest object across all colors
                if area > max_area:
                    max_area = area
                    largest_obj = obj
                    detected_color = color

    return largest_obj, detected_color


def cameraTimerCallback():
    global current_state
    global missedDetections
    global target_color

    # If we're approaching a specific color, track only that color
    if current_state == ROBOT_APPROACHING and target_color:
        objects = ai_vision_15.take_snapshot(target_color)
        if objects:
            handleObjectDetection(ai_vision_15.largest_object(), target_color)
        else:
            # Don't stop arm in approaching mode - let it be controlled by tracking
            print(
                "lost {} object".format(color_names.get(target_color.id, "unknown")),
                end="\r",
            )
            missedDetections = missedDetections + 1
    else:
        # In searching mode, look for any of the three colors
        obj, color = detectAllColors()
        if obj and color:
            target_color = color
            handleObjectDetection(obj, color)
        else:
            # Keep arm at search position when no objects detected in searching mode
            if current_state == ROBOT_SEARCHING:
                moveArmToSearchPosition()
            print("no objects detected", end="\r")
            missedDetections = missedDetections + 1

    # restart the timer
    if current_state != ROBOT_IDLE:
        cameraTimer.event(cameraTimerCallback, cameraInterval)

height_scalar=0.4
area_scalar=0
def handleObjectDetection(obj, color):
    global current_state, missedDetections, knock_turn_time, arm_knock_turn_time, target_color

    area_pct = (obj.height * obj.width) / (CAM_HEIGHT * CAM_WIDTH)
    cy = obj.centerY
    cx = obj.centerX

    # Display detection info with color
    brain.screen.print_at(
        "color: {}".format(color_names.get(color.id, "?")), x=100, y=80
    )
    brain.screen.print_at("area: {:.2f}".format(area_pct), x=100, y=100)
    brain.screen.print_at("height: {}".format(obj.height), x=100, y=120)
    brain.screen.print_at("width: {}".format(obj.width), x=100, y=140)
    brain.screen.print_at("cy: {}".format(cy), x=100, y=160)

    if current_state == ROBOT_SEARCHING:
        print("SEARCHING -> APPROACHING {}".format(color_names.get(color.id, "object")))
        current_state = ROBOT_APPROACHING
        target_color = color  # Lock onto this color
        imuTimer.clear()  # Stop the forward movement timer

    # Not elif, because we want the logic to cascade
    if current_state == ROBOT_APPROACHING:
        # horizontal control
        target_y = CAM_HEIGHT / 2
        K_y = 0.5

        error = cy - target_y
        if area_pct > AREA_THRESHOLD:
            knock_turn_time = knock_timer.time()
        knock_check = knock_timer.time() - knock_turn_time < KNOCK_TIME
        if knock_check:
            error += 600
        turn_effort = K_y * error
        brain.screen.print_at("turn: {:.1f}".format(turn_effort), x=100, y=180)

        left_motor.spin(FORWARD, BASE_SPEED + turn_effort)
        right_motor.spin(FORWARD, BASE_SPEED - turn_effort)

        target_x = (0.5 - (0.2 * area_pct)) * CAM_WIDTH
        K_x = 0.5

        error = cx - target_x
        if area_pct > ARM_AREA_THRESHOLD:
            arm_knock_turn_time = knock_timer.time()
            sleep(int(500), TimeUnits.MSEC)

        arm_knock_check = knock_timer.time() - arm_knock_turn_time < KNOCK_TIME
        if arm_knock_check or knock_check:
            error = 0
        turn_effort = K_x * error
        arm_motor.spin(FORWARD, turn_effort)

        if knock_check:
            sleep(int(KNOCK_TIME * 1000), TimeUnits.MSEC)

    # reset the time out timer
    missedDetections = 0


def handleBoxPickup():
    """Drive backward until a box is detected (< 40mm), then clamp and turn 180°."""
    global current_state

    print("Starting box pickup routine...")
    # Move backward
    left_motor.spin(REVERSE, 100, RPM)
    right_motor.spin(REVERSE, 100, RPM)

    # Keep moving until box is close
    # while True:
    wait(1000, MSEC)
    while ultrasonic_sensor.distance(MM) > 40:
        print(ultrasonic_sensor.distance(MM))
        wait(10, MSEC)

    # Stop and clamp
    left_motor.stop()
    right_motor.stop()
    print("Box detected! Clamping...")
    clamp_motor.spin_for(
        FORWARD, -1800, DEGREES, 100, RPM, wait=False
    )  # adjust spin amount if needed
    wait(1000, MSEC)
    clamp_motor.stop(HOLD)

    # Rotate 180 degrees
    print("Turning 180 degrees...")
    left_motor.spin(FORWARD, 150, RPM)
    right_motor.spin(REVERSE, 150, RPM)
    wait(1400, MSEC)  # Adjust for your robot's rotation speed
    left_motor.stop()
    right_motor.stop()

    # Transition to searching state
    print("Pickup complete -> SEARCHING")
    current_state = ROBOT_SEARCHING

    # Move arm to search position
    moveArmToSearchPosition()

    # Start forward movement and camera detection
    imuTimer.event(handleForward, 100)
    cameraTimer.event(cameraTimerCallback, cameraInterval)


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


def handleButton():
    global current_state, target_color

    if current_state == ROBOT_IDLE:
        print("IDLE -> PICKUP_BOX")
        current_state = ROBOT_PICKUP_BOX
        target_color = None

    # if current_state == ROBOT_IDLE:
    #     print("IDLE -> SEARCHING")
    #     current_state = ROBOT_SEARCHING
    #     target_color = None  # Clear any previous target
    #     # Move arm to search position
    #     moveArmToSearchPosition()
    #     # Start forward movement with IMU control
    #     imuTimer.event(handleForward, 100)
    #     # Start the timer for the camera
    #     cameraTimer.event(cameraTimerCallback, cameraInterval)

    else:  # failsafe; go to IDLE from any other state when button is pressed
        print(" -> IDLE")
        current_state = ROBOT_IDLE
        target_color = None
        left_motor.stop()
        right_motor.stop()
        arm_motor.stop()
        imuTimer.clear()  # Clear IMU timer


handleButton()
def decrease_height():
    global height_scalar
    height_scalar -= 0.1
    print("height_scalar:")
    print(height_scalar)

def increase_height():
    global height_scalar
    height_scalar += 0.1
    print("height_scalar:")
    print(height_scalar)

def decrease_area():
    global area_scalar
    area_scalar -= 0.1
    print("area_scalar:")
    print(area_scalar)

def increase_area():
    global area_scalar
    area_scalar += 0.1
    print("area_scalar:")
    print(area_scalar)

controller.buttonL1.pressed(decrease_height)
controller.buttonR1.pressed(increase_height)
controller.buttonL2.pressed(decrease_area)
controller.buttonR2.pressed(increase_area)
# Main loop
while True:
    # Display current state and sensor info
    brain.screen.print_at("State: {}".format(current_state), x=10, y=20)
    brain.screen.print_at(
        "Sonar: {} mm".format(range_finder_e.distance(MM)), x=10, y=40
    )
    if target_color:
        brain.screen.print_at(
            "Tracking: {}".format(color_names.get(target_color.id, "?")), x=10, y=60
        )

    if current_state == ROBOT_PICKUP_BOX:
        handleBoxPickup()

    # Check for wall detection only during searching state
    if current_state == ROBOT_SEARCHING:
        if checkUltrasonic():
            current_time = brain.timer.time(MSEC)
            # Only turn if enough time has passed since last turn
            if current_time - last_turn_time >= TURN_COOLDOWN:
                handle90DegreeTurn()

    # Check if object is lost during approaching
    if checkForLostObject():
        handleLostObject()

    wait(20, MSEC)  # Small delay to prevent CPU overload