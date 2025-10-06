# region VEXcode Generated Robot Configuration
from vex import *

# Brain should be defined by default
brain = Brain()

# Robot configuration code - Port Bindings
left_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, not False)
right_motor = Motor(Ports.PORT9, GearSetting.RATIO_18_1, not True)
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
KNOCK_TIME = 0.8
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
ROBOT_DRIVING = 5  # <-- NEW STATE for manual control

# Initialize to IDLE state
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
#   Project:      Lab 6 with State Switching and Manual Drive
#   Author:       Modified from VEX
#   Description:  Adds a manual driving state toggled by a controller button.
#
# ------------------------------------------

# Timer for IMU-based forward movement
imuTimer = Timer()

# Timer for camera processing
cameraInterval = 50
cameraTimer = Timer()


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


def handleForward():
    if current_state != ROBOT_SEARCHING:
        return
    angle = brain_inertial.rotation()
    left_motor.spin(FORWARD, baseSpeed - (forwardkd * angle))
    right_motor.spin(FORWARD, baseSpeed + (forwardkd * angle))
    imuTimer.event(handleForward, 100)


def moveArmToSearchPosition():
    arm_motor.spin_to_position(ARM_SEARCH_POSITION, DEGREES, ARM_SPEED, RPM, False)


missedDetections = 0


def checkForLostObject():
    return missedDetections > 20


def handleLostObject():
    global current_state, target_color
    if current_state == ROBOT_APPROACHING:
        print(
            "APPROACHING -> SEARCHING (lost {})".format(
                color_names.get(target_color.id if target_color else 0, "object")
            )
        )
        current_state = ROBOT_SEARCHING
        target_color = None
        moveArmToSearchPosition()
        imuTimer.event(handleForward, 100)


def detectAllColors():
    largest_obj = None
    detected_color = None
    max_area = 0
    for color in color_priority:
        objects = ai_vision_15.take_snapshot(color)
        if objects:
            obj = ai_vision_15.largest_object()
            if obj:
                area = obj.height * obj.width
                if area > max_area:
                    max_area = area
                    largest_obj = obj
                    detected_color = color
    return largest_obj, detected_color


def cameraTimerCallback():
    global current_state, missedDetections, target_color
    if current_state == ROBOT_APPROACHING and target_color:
        objects = ai_vision_15.take_snapshot(target_color)
        if objects:
            handleObjectDetection(ai_vision_15.largest_object(), target_color)
        else:
            print(
                "lost {} object".format(color_names.get(target_color.id, "unknown")),
                end="\r",
            )
            missedDetections += 1
    else:
        obj, color = detectAllColors()
        if obj and color:
            target_color = color
            handleObjectDetection(obj, color)
        else:
            if current_state == ROBOT_SEARCHING:
                moveArmToSearchPosition()
            print("no objects detected", end="\r")
            missedDetections += 1
    if current_state != ROBOT_IDLE and current_state != ROBOT_DRIVING:
        cameraTimer.event(cameraTimerCallback, cameraInterval)


def handleObjectDetection(obj, color):
    global current_state, missedDetections, knock_turn_time, arm_knock_turn_time, target_color
    area_pct = (obj.height * obj.width) / (CAM_HEIGHT * CAM_WIDTH)
    cy = obj.centerY
    cx = obj.centerX
    brain.screen.print_at(
        "color: {}".format(color_names.get(color.id, "?")), x=100, y=80
    )
    brain.screen.print_at("area: {:.2f}".format(area_pct), x=100, y=100)
    if current_state == ROBOT_SEARCHING:
        print("SEARCHING -> APPROACHING {}".format(color_names.get(color.id, "object")))
        current_state = ROBOT_APPROACHING
        target_color = color
        imuTimer.clear()
    if current_state == ROBOT_APPROACHING:
        target_y = CAM_HEIGHT / 2
        K_y = 0.5
        error = cy - target_y
        if area_pct > AREA_THRESHOLD:
            knock_turn_time = knock_timer.time()
        knock_check = knock_timer.time() - knock_turn_time < KNOCK_TIME
        if knock_check:
            error += 600
        turn_effort = K_y * error
        left_motor.spin(FORWARD, BASE_SPEED + turn_effort)
        right_motor.spin(FORWARD, BASE_SPEED - turn_effort)
        target_x = (0.65 - (0.0 * area_pct)) * CAM_WIDTH
        K_x = 0.5
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
    missedDetections = 0


def handleBoxPickup():
    global current_state
    print("Starting box pickup routine...")
    left_motor.spin(REVERSE, 100, RPM)
    right_motor.spin(REVERSE, 100, RPM)
    wait(1000, MSEC)
    while ultrasonic_sensor.distance(MM) > 40:
        print(ultrasonic_sensor.distance(MM))
        wait(10, MSEC)
    left_motor.stop()
    right_motor.stop()
    print("Box detected! Clamping...")
    clamp_motor.spin_for(FORWARD, -1800, DEGREES, 100, RPM, wait=True)
    clamp_motor.stop(HOLD)
    print("Turning 180 degrees...")
    left_motor.spin(FORWARD, 150, RPM)
    right_motor.spin(REVERSE, 150, RPM)
    wait(1400, MSEC)
    left_motor.stop()
    right_motor.stop()
    print("Pickup complete -> SEARCHING")
    current_state = ROBOT_SEARCHING
    moveArmToSearchPosition()
    imuTimer.event(handleForward, 100)
    cameraTimer.event(cameraTimerCallback, cameraInterval)


def handle90DegreeTurn():
    global current_state, last_turn_time
    if current_state == ROBOT_SEARCHING:
        print("Performing 90-degree turn")
        imuTimer.clear()
        left_motor.stop()
        right_motor.stop()
        left_motor.spin(FORWARD, 150, RPM)
        right_motor.spin(REVERSE, 150, RPM)
        wait(700, MSEC)
        left_motor.stop()
        right_motor.stop()
        brain_inertial.set_rotation(brain_inertial.rotation() + 90)
        last_turn_time = brain.timer.time(MSEC)
        wait(200, MSEC)
        imuTimer.event(handleForward, 100)
        print("Turn complete, resuming search")


def handleDriving():
    """NEW: Control the robot using Split Arcade Drive on the controller."""
    # Right joystick Y-axis for forward/backward, Left joystick X-axis for turning
    forward_speed = controller.axis2.position()
    turn_speed = controller.axis4.position()

    # Calculate motor speeds
    left_speed = forward_speed + turn_speed
    right_speed = forward_speed - turn_speed

    # Apply speeds to motors
    left_motor.spin(FORWARD, left_speed, PERCENT)
    right_motor.spin(FORWARD, right_speed, PERCENT)

    # Manual arm control with shoulder buttons
    if controller.buttonL1.pressing():
        arm_motor.spin(FORWARD, 75, PERCENT)
    elif controller.buttonL2.pressing():
        arm_motor.spin(REVERSE, 75, PERCENT)
    else:
        arm_motor.stop(BRAKE)

    # Manual clamp control with shoulder buttons
    if controller.buttonR1.pressing():
        clamp_motor.spin(FORWARD, 75, PERCENT)
    elif controller.buttonR2.pressing():
        clamp_motor.spin(REVERSE, 75, PERCENT)
    else:
        clamp_motor.stop(HOLD)


def handleButton():
    """Handles state transitions based on the current state."""
    global current_state, target_color

    # Case 1: Start the initial pickup sequence from IDLE
    if current_state == ROBOT_IDLE:
        print("IDLE -> PICKUP_BOX")
        current_state = ROBOT_PICKUP_BOX
        return

    # Case 2: Switch from Manual Driving to Autonomous Searching
    if current_state == ROBOT_DRIVING:
        print("DRIVING -> SEARCHING")
        left_motor.stop()
        right_motor.stop()
        current_state = ROBOT_SEARCHING
        target_color = None
        moveArmToSearchPosition()
        imuTimer.event(handleForward, 100)
        cameraTimer.event(cameraTimerCallback, cameraInterval)

    # Case 3: Switch from any Autonomous mode (Searching/Approaching) to Manual Driving
    elif current_state == ROBOT_SEARCHING or current_state == ROBOT_APPROACHING:
        print("AUTONOMOUS -> DRIVING")
        # Stop all autonomous movement
        left_motor.stop()
        right_motor.stop()
        arm_motor.stop()
        # Clear autonomous timers
        imuTimer.clear()
        cameraTimer.clear()
        current_state = ROBOT_DRIVING
        target_color = None


# Bind the state-switching function to a controller button
controller.buttonA.pressed(handleButton)

# Main loop
while True:
    # Display current state on brain screen
    state_names = {
        0: "IDLE",
        1: "SEARCHING",
        2: "APPROACHING",
        3: "TURN",
        4: "PICKUP_BOX",
        5: "DRIVING",
    }
    brain.screen.print_at(
        "State: {}".format(state_names.get(current_state, "Unknown")), x=10, y=20
    )
    brain.screen.print_at(
        "Sonar: {} mm".format(range_finder_e.distance(MM)), x=10, y=40
    )
    if target_color:
        brain.screen.print_at(
            "Tracking: {}".format(color_names.get(target_color.id, "?")), x=10, y=60
        )

    # State-dependent actions
    if current_state == ROBOT_PICKUP_BOX:
        handleBoxPickup()  # This function will transition state to SEARCHING

    elif current_state == ROBOT_DRIVING:
        handleDriving()  # Call the manual drive function

    elif current_state == ROBOT_SEARCHING:
        if checkUltrasonic():
            current_time = brain.timer.time(MSEC)
            if current_time - last_turn_time >= TURN_COOLDOWN:
                handle90DegreeTurn()

    # Check for lost object only in approaching state
    if current_state == ROBOT_APPROACHING and checkForLostObject():
        handleLostObject()

    wait(20, MSEC)
