package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EagleMatrix.AutoDriver;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Position;

@Autonomous(name = "Auto", group = Constants.GroupNames.Autonomous, preselectTeleOp = "TeleOp")
public class Auto extends LinearOpMode {
    private Robot robot;
    private GoBildaPinpointDriver odometry;
    private AutoDriver autoDriver;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        odometry = robot.odometry;
        autoDriver = robot.autoDriver;

        waitForStart();

        while (autoDriver.moveTo(new Position(0, 10, 0, AngleUnit.DEGREES), 0.5) && opModeIsActive()) {
            codeLoop();
        }
        terminateOpModeNow();
    }

    public void codeLoop(){
        telemetry.addLine("Position: " + odometry.getPosition());
    }
}
