package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = Constants.GroupNames.TeleOp)
public class TeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        // Drive the robot with the game-pad
        robot.drive.driveMecanumFieldCentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Reset IMU for Field Centric
        if (gamepad1.left_bumper) {
            robot.compass.resetYaw();
        }

        // Set speed mode
        if (gamepad1.right_bumper) {
            robot.drive.setPower(1);
        } else {
            robot.drive.setPower(0.6);
        }

        // Front Wrist
        if (gamepad2.x){
            robot.intake.wristUp();
        } else if (gamepad2.b) {
            robot.intake.wristDown();
        }

        // Back Wrist
        if (gamepad2.y) {
            robot.intake.wristBackUp();
        } else if (gamepad2.a) {
            robot.intake.wristBackDown();
        }

        // Lift
        robot.lift.liftMove(gamepad2.left_stick_y);

        // Lift Tilt
        if (gamepad2.left_stick_x == 1){
            robot.lift.liftTiltBack();
        } else if (gamepad2.left_stick_x == -1){
            robot.lift.liftTIltStraight();
        }

        // Shoulder
        robot.lift.shoulderMove(gamepad2.right_stick_x);

        // Hang Hooks
        if (gamepad2.dpad_up) {
            robot.lift.hang(1, 0);
        } else if (gamepad2.dpad_down) {
            robot.lift.hang(0.1, 0.7);
        }

        //  Front Claw
        if (gamepad2.right_trigger > 0) {
            robot.intake.clawOpen();
        } else {
            robot.intake.clawClose();
        }

        // Back Claw
        if (gamepad2.left_trigger > 0) {
            robot.intake.clawBackOpen();
        } else {
            robot.intake.clawBackClose();
        }

        // spin Back Claw
        if (gamepad2.dpad_left) {
            robot.intake.spinWristStraight();
        } else if (gamepad2.dpad_right) {
            robot.intake.spinWristSideways();
        }
        // Elbow Motor
        if (gamepad2.right_stick_y > 0) {
            robot.intake.ElbowMotorUp();
        } else if (gamepad2.right_stick_y < 0) {
            robot.intake.ElbowMotorDown();
        }

        robot.odometry.update();

        // Telemetry
        telemetry.addLine(robot.lift.getTelemetry());
        telemetry.addLine(robot.lift.getJointLiftPosition());
        telemetry.addLine(robot.intake.getTelemetry());
        telemetry.addLine(robot.odometry.getPosition().toString());
        telemetry.addLine(robot.compass.getTelemetry());
    }
}
