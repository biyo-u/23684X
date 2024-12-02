package org.firstinspires.ftc.teamcode.EagleMatrix;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * DriveMovements is where you build the drivetrain movements based on motor direction.
 */

public class DriveMovements {
    Robot robot;
    double modifier = 0.6;
    double rest = 0;

    /**

     */
    public DriveMovements(Robot robot){
        this.robot = robot;
    }
    /**
     * Eagleflow is a method that determines power of the motors based on direction. It will power the motors in order to achieve a desired movement, such as strafing, rotation, driving, and diagonal movement.
     * <p>
     * Strafe Left: FL = -1, FR = 1, RL = 1, RR = -1
     *  <p>
     * Strafe Right: FL = 1, FR = -1, RL = -1, RR = 1
     * <p>
     * Angle Strafe Forward Right: FL = 1, FR = 0, RL = 0, RR = 1
     * <p>
     * Angle Strafe Forward Left: FL = 0, FR = 1, RL = 1, RR = 0
     * <p>
     * Angle Strafe Backward Right: FL = -1, FR = 0, RL = 0, RR = -1
     * <p>
     * Angle Strafe Backward Left: FL = 0, FR = -1, RL = -1, RR = 0
     * <p>
     * Rotate Clockwise: FL = 1, FR = -1, RL = 1, RR = -1
     * <p>
     * Rotate Counter-clockwise: FL = -1, FR = 1, RL = -1, RR = 1
     * <p>
     * Stop: FL = 0, FR = 0, RL = 0, RR = 0
     * @param X The power applied to the motors, X
     * @param Y The power applied to the motors, Y
     * @param TARGETHEADING The power applied to the motors, TARGETHEADING
     */
    public void EagleFlow(double X, double Y, double TARGETHEADING){
        X = X * modifier;
        Y = -Y * modifier;
        TARGETHEADING = TARGETHEADING * modifier;
        double CURRENTHEADING = robot.compass.getHeading();

        double rotX = X * Math.cos(-CURRENTHEADING) - Y * Math.sin(-CURRENTHEADING);
        double rotY = X * Math.sin(-CURRENTHEADING) + Y * Math.cos(-CURRENTHEADING);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(TARGETHEADING), 1);
        double frontLeftPower = (Y + X + TARGETHEADING) / denominator;
        double backLeftPower = (Y - X + TARGETHEADING) / denominator;
        double frontRightPower = (Y - X - TARGETHEADING) / denominator;
        double backRightPower = (Y + X - TARGETHEADING) / denominator;

        robot.drive.getFrontLeft().setPower(frontLeftPower);
        robot.drive.getFrontRight().setPower(frontRightPower);
        robot.drive.getRearLeft().setPower(backLeftPower);
        robot.drive.getRearRight().setPower(backRightPower);
    }
    // Drivetrain stops
    public void EagleStop(){
        robot.drive.getFrontLeft().setPower(rest);
        robot.drive.getFrontRight().setPower(rest);
        robot.drive.getRearLeft().setPower(rest);
        robot.drive.getRearRight().setPower(rest);
    }
}
