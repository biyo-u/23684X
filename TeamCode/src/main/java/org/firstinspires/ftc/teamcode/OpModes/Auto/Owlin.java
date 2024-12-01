package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EagleMatrix.DrivetrainMovements;
import org.firstinspires.ftc.teamcode.EagleMatrix.DrivetrainMovements.MotorDirection;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "OwlinDrive", group = Constants.GroupNames.Autonomous,preselectTeleOp = "TeleOp")
public class Owlin extends OpMode {

    private Robot robot;
    DrivetrainMovements drivetrainMovements;

    double modifier = 0.6;
    double forward = 1 * modifier;
    double backward = -1 * modifier;
    double rest = 0;
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        this.drivetrainMovements = new DrivetrainMovements(new Robot(hardwareMap));
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            drivetrainMovements.move(MotorDirection.FORWARD);
        } else {
            drivetrainMovements.move(MotorDirection.STOP);
        }

        robot.odometry.update();

        telemetry.addLine(robot.odometry.getTelemetry());
        telemetry.addLine(robot.compass.getTelemetry());
    }
}
