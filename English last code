from pybricks.hubs import * # Import the hub library
from pybricks.pupdevices import * # Import the peripheral devices library
from pybricks.parameters import * # Import the parameter constants
from pybricks.tools import * # Import the general tools library
 
hub = PrimeHub() # Names the hub as hub
left_motor = Motor(Port.B) # The motor in port B is named left_motor and rotates clockwise
right_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE) # The motor in port F is named right_motor and rotates counterclockwise
attachment_left = Motor(Port.E) # The attachment motor in port E is named attachment_left and the gear ratio is 12, 12, 20
attachment_right = Motor(Port.A)  # The attachment motor in port A is named attachment_right and the gear ratio is 12, 12, 20
attachment_left.control.limits(1000, 6500) # Set speed and acceleration limits for the left attachment motor
attachment_right.control.limits(1000, 6500) # Set speed and acceleration limits for the right attachment motor
left_motor.control.limits(2000, 5500) # Set speed and acceleration limits for the left drive motor
right_motor.control.limits(2000, 5500) # Set speed and acceleration limits for the right drive motor
while not hub.imu.ready(): hub.display.char("x") # Wait until the gyro is ready and display an x
 
hub.imu.reset_heading(0) # Sets the gyro value to 0
heading = 0 # Creates a variable named heading with a value of 0
right_gear_ratio = 1 # Creates a variable named right_gear_ratio with a value of 1, this will count the gear ratios
left_gear_ratio = 1 # Creates a variable named left_gear_ratio with a value of 1, this will count the gear ratios
 
def straight(s_distance, s_min_speed=40, s_acceleration=40, s_correction=0.01, s_max_speed = 700, s_deceleration=80, s_timeout = None): # Creates a function named straight with parameters for s_distance and movement control
    if s_timeout != None: # Check if a s_timeout limit is provided
        s_timeout_watch = StopWatch() # Create a stopwatch object for the s_timeout
        s_timeout_watch.resume() # Start the stopwatch
    global heading # Allows the function to use the heading variable inside the function
    left_motor.reset_angle(0) # Sets the left angle to 0
    right_motor.reset_angle(0) # Sets the right angle to 0
    s_distance = s_distance / 0.489 # Converts s_distance given in mm to motor degrees, dividing wheel circumference by 360
    s_acceleration /= 0.489 # Converts s_acceleration given in mm to motor degrees, dividing wheel circumference by 360
    s_deceleration /= 0.489 # Converts s_deceleration given in mm to motor degrees, dividing wheel circumference by 360
    while True: # Starts a loop that runs until stopped
        if s_timeout != None and s_timeout_watch.time() >= s_timeout: break # Exit the loop if the time limit is reached           
        s_distance_done = (left_motor.angle()+right_motor.angle()) / 2 # Adds left and right angles and divides by 2, resulting in the average s_distance covered
        s_distance_left = s_distance - s_distance_done # Subtracts s_distance_done from s_distance to get s_distance_left
        s_multiplier = s_distance_left/abs(s_distance_left) # Divides s_distance_left by its absolute value to get the direction (1 or -1)
        s_distance_done = abs(s_distance_done) # The absolute value of s_distance_done should be s_distance_done
        s_distance_left = abs(s_distance_left) # The absolute value of s_distance_left should be s_distance_left
        if s_distance_left < 3: # If s_distance_left is less than 3 motor degrees
            break # Then stop, i.e., exit the while loop
        if s_distance_left < s_deceleration : # If s_distance_left is less than s_deceleration (if already in the s_deceleration phase)
            s_ratio = s_distance_left / s_deceleration # Then the s_deceleration s_ratio is s_distance_left/s_deceleration (0-1)
            s_current_speed = max(s_ratio * s_max_speed, s_min_speed) * s_multiplier # Calculate speed for s_deceleration multiplied by the direction
        elif s_distance_done < s_acceleration: # If not the previous, and s_distance_done is less than s_acceleration
            s_ratio = s_distance_done / s_acceleration # The s_ratio equals s_distance_done / s_acceleration (0-1)
            s_current_speed = max(s_ratio * s_max_speed, s_min_speed) * s_multiplier # Calculate speed for s_acceleration multiplied by the direction
        else: # If none of the above (constant speed phase)
            s_current_speed = s_max_speed * s_multiplier # s_current_speed equals s_max_speed multiplied by the direction
        s_heading_error = (heading - hub.imu.heading()) * s_correction # Calculate heading error multiplied by the s_correction factor
        s_correction_amount = s_current_speed * s_heading_error * s_multiplier # Calculate the amount to correct by based on speed and error
        left_motor.run (s_current_speed - s_correction_amount) # Apply speed to left motor with s_correction
        right_motor.run(s_current_speed + s_correction_amount) # Apply speed to right motor with s_correction
    left_motor.hold() # Left stops (holds its position right there)
    right_motor.hold()# Right stops (holds its position right there)
 
def turn(t_target_angle, t_max_speed=360, t_deceleration=80, t_min_speed=50, t_timeout = None): # Creates a function named turn with parameters for angle and speed control
    if t_timeout != None: # Check if a t_timeout limit is provided
        t_timeout_watch = StopWatch() # Create a stopwatch object for the t_timeout
        t_timeout_watch.resume() # Start the stopwatch
    t_base_angle = hub.imu.heading() # t_base_angle equals the gyro value, needed to correct based on previous errors
    global heading # Allows the function to use the heading variable inside the function
    t_target_heading = heading + t_target_angle # t_target_heading equals heading plus the requested turn angle
    heading = t_target_heading # Heading equals t_target_heading
    t_target_heading -= t_base_angle # Subtract t_base_angle from t_target_heading to handle relative turn
    while True: # Loop runs until exit
        if t_timeout != None and t_timeout_watch.time() >= t_timeout: break # Exit the loop if the time limit is reached
        t_angle_done = hub.imu.heading() - t_base_angle # t_angle_done equals current gyro minus the base angle
        t_angle_left = t_target_heading - t_angle_done # t_angle_left equals t_target_heading minus t_angle_done
        t_multiplier = t_angle_left/abs(t_angle_left) # Determine turn direction (1 or -1)
        t_angle_left = abs(t_angle_left) # The absolute value of t_angle_left should be t_angle_left
        if t_angle_left <= 0.5: # If t_angle_left is less than or equal to 0.5
            break # Then stop, i.e., exit the while loop      
        if t_angle_left < t_deceleration : # If t_angle_left is less than t_deceleration (t_deceleration phase)
            t_ratio = t_angle_left / t_deceleration # Then the t_deceleration t_ratio is t_angle_left/t_deceleration
            t_current_speed = max(t_ratio * t_max_speed, t_min_speed) * t_multiplier # Calculate current turning speed
        else: # If not the previous, i.e., not decelerating
            t_current_speed = t_max_speed * t_multiplier # Then t_current_speed equals t_max_speed multiplied by direction
        left_motor.run(-t_current_speed) # Spin left motor in the calculated direction
        right_motor.run(t_current_speed) # Spin right motor in the opposite direction
    left_motor.hold() # Left stops (holds its position)
    right_motor.hold() # Right stops (holds its position)
    wait(100) # Wait for 100 milliseconds
 
def attachment_right_move(a_angle, a_speed = 400, a_timeout=None): # Creates a function named attachment_right_move for the right motor
    if a_timeout != None: # Check if a a_timeout limit is provided
        a_timeout_watch = StopWatch() # Create a stopwatch object for the a_timeout
        a_timeout_watch.resume() # Start the stopwatch
        attachment_right.run_angle(a_speed, a_angle * left_gear_ratio, wait=False) # Start moving right attachment in the background
        while abs(attachment_right.angle()-(a_angle * right_gear_ratio)) > 1: # Loop until target angle is reached
            if a_timeout_watch.time() >= a_timeout: break # Break if a_timeout occurs
    else: # If no a_timeout is specified
        attachment_right.run_angle(a_speed, a_angle * left_gear_ratio) # Runs attachment_right in degrees and waits for completion

def attachment_left_move(a_angle, a_speed = 400, a_timeout = None): # Creates a function named attachment_left_move for the left motor
    if a_timeout != None: # Check if a a_timeout limit is provided
        a_timeout_watch = StopWatch() # Create a stopwatch object for the a_timeout
        a_timeout_watch.resume() # Start the stopwatch
        attachment_left.run_angle(a_speed, a_angle * left_gear_ratio, wait=False) # Start moving left attachment in the background
        while abs(attachment_left.angle()-(a_angle * left_gear_ratio)) > 1: # Loop until target angle is reached
            if a_timeout != None and a_timeout_watch.time() >= a_timeout: break # Break if a_timeout occurs
    else: # If no a_timeout is specified
        attachment_left.run_angle(a_speed, a_angle * left_gear_ratio) # Runs attachment_left in degrees and waits for completion
        
def attachment_right_background(a_angle, a_speed = 400): # Creates a function named attachment_right_background for non-blocking movement
    attachment_right.run_angle(a_speed, a_angle * right_gear_ratio, wait=False) # Runs attachment_right in degrees and does not wait
 
def attachment_left_background(a_angle, a_speed = 400): # Creates a function named attachment_left_background for non-blocking movement
    attachment_left.run_angle(a_speed, a_angle * left_gear_ratio, wait=False) # Runs attachment_left in degrees and does not wait
 
hub.system.set_stop_button(Button.BLUETOOTH) # Sets the stop button to the bluetooth button
hub.display.number(1) # Displays number 1, as run 1 starts
voltage = hub.battery.voltage() # Gets the battery voltage
print(voltage) # Prints the battery voltage to the console
 
def run_1(): # Creates a function named run_1
    hub.imu.reset_heading(0) # Sets gyro value to 0
    wait(200) # Wait for 200 milliseconds
 
def run_2(): # Creates a function named run_2
    hub.imu.reset_heading(0) # Sets gyro value to 0
    wait(200) # Wait for 200 milliseconds
 
def run_3(): # Creates a function named run_3
    hub.imu.reset_heading(0) # Sets gyro value to 0
    wait(200) # Wait for 200 milliseconds
    
def run_4(): # Creates a function named run_4
    hub.imu.reset_heading(0) # Sets gyro value to 0
    wait(200) # Wait for 200 milliseconds

current_run = 0 # Creates a variable named current_run with a value of 0, which counts the run number
runs = [run_1, run_2, run_3, run_4] # Creates an array named runs and specifies its elements
max_runs = len(runs) # Creates max_runs and its value is the number of elements in runs
 
while True: # A loop that runs until we exit
    hub.display.number(current_run + 1) # Hub displays current_run + 1
    pressed = [] # Creates an array named pressed and sets it to empty
    while not any(pressed): # As long as there's nothing in the pressed array, run
        pressed = hub.buttons.pressed() # The pressed array contains the buttons pressed on the hub  
    hold_timer = StopWatch() # Creates the hold_timer variable, starting the stopwatch
    vibrated = False # Creates a variable named vibrated and sets it to False
    while hub.buttons.pressed(): # A loop that runs as long as a button is pressed
        if hold_timer.time() > 500: # If the pressed time is more than half a second
            vibrated = True # Vibrated becomes True
            attachment_left_background(45, speed=900) # Starts attachment_left_background move for calibration
            attachment_right_move(45, speed=900) # Starts attachment_right_move for calibration
            attachment_left_background(-45, speed=900) # Calibrates left attachment back
            attachment_right_move(-45, speed=900) # Calibrates right attachment back
        pass # Continue waiting
    if vibrated: # If it already vibrated
        continue  # Skip to the next loop iteration
 
    if Button.RIGHT in pressed: # If the right button is in pressed
        current_run = (current_run + 1) % max_runs # Current_run equals current_run + 1 remainder of max_runs
    if Button.LEFT in pressed: # If the left button is in pressed
        current_run = (current_run - 1) % max_runs # Current_run equals current_run - 1 remainder of max_runs
    if Button.CENTER in pressed: # If the center button is in pressed
        heading = 0 # Resets the target heading to 0
        hub.imu.reset_heading(0) # Sets the gyro value to 0
        try: # Try to run the code, go to except if interrupted
            hub.system.set_stop_button(Button.CENTER) # The stop button should be the center button
            runs[current_run]() # Runs the current run element of runs
            current_run = (current_run + 1) % max_runs # Increment the run selection
        except SystemExit: # If trying to exit, do this
            while Button.CENTER in hub.buttons.pressed(): # As long as the center button is pressed
                pass # Wait for button release
        left_motor.stop() # Left motor stops
        right_motor.stop() # Right motor stops
        attachment_left.stop() # Left attachment motor stops
        attachment_right.stop() # Right attachment motor stops
        hub.system.set_stop_button(Button.BLUETOOTH) # Reset the stop button to bluetooth
