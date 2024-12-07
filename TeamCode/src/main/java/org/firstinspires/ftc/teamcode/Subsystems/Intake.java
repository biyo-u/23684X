package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

public class Intake {
    private final Servo clawFront;
    private final Servo wristServo;
    private final Servo clawBack;
    private final Servo spinWrist;
    private final DcMotor elbowMotor;

    /**
     * Constructor for the Intake subsystem.
     *
     * @param clawFront  The Servo object representing the claw servo.
     * @param wristServo The servo object representing the claw lift.
     * @param clawBack The spinning claw servo.
     * @param spinWrist     The second wrist Servo.
     */
    public Intake(Servo clawFront, Servo wristServo, Servo clawBack, Servo spinWrist, DcMotor elbowMotor) {
        this.clawFront = clawFront;
        this.wristServo = wristServo;
        this.clawBack = clawBack;
        this.spinWrist = spinWrist;
        this.elbowMotor = elbowMotor;
    }

    /**
     * Opens the claw of the robot.
     * <p>
     * This method sets the position of the claw servo to 0, which corresponds to the open position.
     */
    public void clawOpen() {
        clawFront.setPosition(0);
    }

    /**
     * Closes the claw of the robot.
     * <p>
     * This method sets the position of the claw servo to 1, which corresponds to the closed position.
     */
    public void clawClose() {
        clawFront.setPosition(1);
    }

    /**
     * Moves the wrist servo to the "up" position.
     * <p>
     * This method sets the position of the wrist servo to 1.
     */
    public void wristUp() {
        wristServo.setPosition(1);
    }

    /**
     * Moves the wrist servo to the "down" position.
     * <p>
     * This method sets the position of the wrist servo to 0.
     */
    public void wristDown() {
        wristServo.setPosition(0);
    }

    public void spinWristSideways(){
        spinWrist.setPosition(1);
    }

    public void spinWristStraight(){
        spinWrist.setPosition(0);
    }

    public void clawBackClose(){
        clawBack.setPosition(0);
    }

    public void clawBackOpen(){
        clawBack.setPosition(1);
    }

    public void ElbowMotorDown(){
        elbowMotor.setPower(1);
    }

    public void ElbowMotorUp(){
        elbowMotor.setPower(-1);
    }

    public String getTelemetry() {
        return String.format(Locale.getDefault(), """
                Front Claw Servo: %f
                Wrist Servo: %f
                Spin Wrist Servo: %f
                Claw Back Servo: %f
                Elbow Motor: %d""", clawFront.getPosition(), wristServo.getPosition(), spinWrist.getPosition(), clawBack.getPosition(), elbowMotor.getCurrentPosition());
    }
}
