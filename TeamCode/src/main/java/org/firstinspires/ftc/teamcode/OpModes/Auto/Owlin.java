package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EagleMatrix.DriveMovements;
import org.firstinspires.ftc.teamcode.EagleMatrix.DriveMovements.MotorDirection;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "OwlinDrive", group = Constants.GroupNames.Autonomous,preselectTeleOp = "TeleOp")
public class Owlin extends OpMode {

    private Robot robot;
    DriveMovements driveMovements;

    double modifier = 0.6;
    double forward = 1 * modifier;
    double backward = -1 * modifier;
    double rest = 0;
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        this.driveMovements = new DriveMovements(new Robot(hardwareMap));
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            driveMovements.move(MotorDirection.FORWARD);
        } else {
            driveMovements.move(MotorDirection.STOP);
        }

        robot.odometry.update();

        telemetry.addLine(robot.odometry.getTelemetry());
        telemetry.addLine(robot.compass.getTelemetry());
    }
}
