package org.firstinspires.ftc.teamcode.EagleMatrix;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * LiftMovements is where you build the intake and lift movements based on motor direction.
 */

public class LiftMovements {

    Robot robot;

    double upward = 1;
    double downward = -1;
    double rest = 0;
    int chambers = 800;
    int lowered = 0;
    int high_basket = 3000;
    int pulled_down = 600;

    public LiftMovements(Robot robot){
        this.robot = robot;
    }


    // LIFT MOVEMENTS

    /**
     * Raises the lift to the position to score on the high chambers.
     */
    public void LiftScoreChamber() {
        // TODO: ENSURE THIS FUNCTIONS!
        robot.lift.getLiftMotorRight().setTargetPosition(chambers);
        robot.lift.getLiftMotorLeft().setTargetPosition(chambers);
        robot.lift.getLiftMotorRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.getLiftMotorLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.liftMove(upward);
    }
    /**
     * Lowers the lift.
     */
    public void LiftLower() {
        robot.lift.getLiftMotorRight().setTargetPosition(lowered);
        robot.lift.getLiftMotorLeft().setTargetPosition(lowered);
        robot.lift.getLiftMotorRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.getLiftMotorLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.liftMove(downward);
    }
    /**
     * Lowers the lift.
     */
    public void LiftPullDown() {
        robot.lift.getLiftMotorRight().setTargetPosition(pulled_down);
        robot.lift.getLiftMotorLeft().setTargetPosition(pulled_down);
        robot.lift.getLiftMotorRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.getLiftMotorLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.liftMove(downward);
    }
    /**
     * Stops and the lift at current position.
     */
    public void LiftRest() {
        robot.lift.liftMove(rest);
    }
    /**
     * Raises the lift to the position to score on the high basket.
     */
    public void LiftScoreBasket() {
        robot.lift.getLiftMotorRight().setTargetPosition(high_basket);
        robot.lift.getLiftMotorLeft().setTargetPosition(high_basket);
        robot.lift.getLiftMotorRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.getLiftMotorLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.liftMove(upward);
    }

    // CLAW MOVEMENTS
    /**
     * Opens the robot's claw.
     * This method calls the `clawOpen()` method of the `intake` object
     * on the robot, which is responsible for physically opening the claw.
     */
    public void ClawOpen() {
        robot.intake.clawOpen();
    }
    /**
     * Closes the robot's claw.
     */
    public void ClawClose() {
        robot.intake.clawClose();
    }

    // WRIST MOVEMENTS
    /**
     * Retracts claw in.
     */
    public void WristIn() {
        robot.intake.wristUp();
    }
    /**
     * Extends claw out.
     */
    public void WristOut() {
        robot.intake.wristDown();
    }

    // SHOULDER MOVEMENTS
    /**
     * Raises arm up.
     */
    public void ShoulderUp() {
        robot.lift.shoulderMove(upward);
    }
    /**
     * Lowers arm down.
     */
    public void ShoulderDown() {
        robot.lift.shoulderMove(downward);
    }

    public void ElbowUp(){
        robot.intake.ElbowMotorUp();
    }

    public void ElbowDown(){
        robot.intake.ElbowMotorDown();
    }

    public void SpinWristStraight(){
        robot.intake.spinWristStraight();
    }

    public void SpinWristSideways() {
        robot.intake.spinWristSideways();
    }

    public void BackClawOpen(){
        robot.intake.clawBackOpen();
    }

    public void BackClawClose(){robot.intake.clawBackClose();}
}