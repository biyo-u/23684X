# 23684X

The FIFTH Tech Titans GitHub repository. Home of EagleMatrix
1. 23684-depreciated
2. 23684RR
3. 23684
4. 23684.TT (deleted)
5. 23684X

*Variables (except enums), Robot config, and functions* are always *camelCase*
*Classes and enums* are always *PascalCase*
Drive motors will be names weather they front or rear + weather they are left or right (like
frontLeft, frontRight, rearLeft, rearRight)
In any order where 4 drive motors are needed, they must be in the order: frontLeft, frontRight,
rearLeft, rearRight
Whenever something is given a power, there should not be a stop() function. Instead, set the power
to 0 to stop it.
If there isn't a way to set power (like moveUp and moveDown) then there should be a stop() function.

### Gamepad 1 (Drive):

- Left Bumper: Reset IMU
- Right Bumper: Speed Mode
- Left Trigger: Nothing
- Right Trigger: Nothing
- Left Joystick: Drive
- Right Joystick: Turn
- DPad: Nothing
- X: Nothing
- Y: Nothing
- A: Nothing
- B: Nothing

### Gamepad 2 (Actions):

- Left Bumper: Nothing
- Right Bumper: Nothing
- Left Trigger: Intake In
- Right Trigger: Intake Out
- Left Joystick: Lift (Up, Down, Tilt Left, and Tilt Right)
- Right Joystick: Shoulder (Left and Right) and Elbow (Up and Down)
- DPad: Nothing
- X: Wrist Down
- Y: Wrist Up
- A: Nothing
- B: Nothing

## EagleMatrix
Is Roadrunner turning out to be a massive headache? Using a GoBildaPinpointDriver and encoders... or drive encoders? Do you need a simpler, easier way to create the basis you need for automated robot drivetrain AND scoring system movement? Well... say hello to EagleMatrix! Unlike Roadrunner, this completely customizable, easy to implement and easy to adapt drivepath system uses prexisting FTC SDK code to create a comprehensive-yet-straight forward library of code you can call upon to create your autonomous! With a modular and adaptive design that you can open up and change to your needs, and the ability to be easily and effectively implemented for BOTH TELEOP AND AUTO OpModes, this is a "keep it simple stupid" solution to Roadrunner. Use an external IMU or odometry pods, AprilTags or other vision processing, and sensor inputs in a clean, effective library. EagleMatrix, flying high. 

*DISCLAIMER: EAGLEMATRIX 0.2.0 IS IN EXTREMELY EARLY DEVELOPMENT. THE VERSION SEEN HERE IS SUBJECT TO CHANGE, A COMPLETE REDESIGN, AND OTHER UPDATES. THIS VERSION IS DEEPLY INTEGRATED WITH THE TECH TITANS CODEBASE, WHILE THE FINAL VERSION WILL ALLOW FOR EASY ADDITION OF AND/OR TO PREEXISTING CODE!*
