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
     * MotorDirection is where you set the direction of the motors based on the desired action.
     */
    public enum MotorDirection {
        FORWARD (1, 1, 1, 1),
        BACKWARD (-1,-1,-1,-1),
        STRAFE_LEFT (-1,1,1,-1),
        STRAFE_RIGHT (1,-1,-1,1),
        ANGLE_STRAFE_FORWARD_RIGHT (1, 0, 0, 1),
        ANGLE_STRAFE_FORWARD_LEFT (0,1,1,0),
        ANGLE_STRAFE_BACKWARD_RIGHT (-1,0,0,-1),
        ANGLE_STRAFE_BACKWARD_LEFT (0,-1,-1,0),
        ROTATE_CLOCKWISE (1,-1,1,-1),
        ROTATE_COUNTERCLOCKWISE (-1,1,-1,1),
        STOP (0,0,0,0);

        private final double FL;
        private final double FR;
        private final double RL;
        private final double RR;

        MotorDirection(double FL, double FR, double RL, double RR) {
            this.FL = FL;
            this.FR = FR;
            this.RL = RL;
            this.RR = RR;
        }
    }

    public DriveMovements(Robot robot){
        this.robot = robot;
    }
    /**
     * move is a method that determines power of the motors based on direction. It will power the motors in order to achieve a desired movement, such as strafing, rotation, driving, and diagonal movement.
     *
     * @param direction The direction of power applied to the motors
     */
    public void move(MotorDirection direction){
        robot.drive.getFrontLeft().setPower(direction.FL * modifier);
        robot.drive.getFrontRight().setPower(direction.FR * modifier);
        robot.drive.getRearLeft().setPower(direction.RL * modifier);
        robot.drive.getRearRight().setPower(direction.RR * modifier);
    }
    // Drivetrain stops
    public void stop(){
        robot.drive.getFrontLeft().setPower(rest);
        robot.drive.getFrontRight().setPower(rest);
        robot.drive.getRearLeft().setPower(rest);
        robot.drive.getRearRight().setPower(rest);
    }
}
