package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EagleMatrix.AutoDriver;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Distance;
import org.firstinspires.ftc.teamcode.Utilities.Position;
import org.firstinspires.ftc.teamcode.Utilities.Rotation;

@Autonomous(name = "LinearAuto", group = Constants.GroupNames.Autonomous, preselectTeleOp = "TeleOp")
public class LinearAuto extends LinearOpMode {
    private Robot robot;
    private GoBildaPinpointDriver odometry;
    private AutoDriver autoDriver;

    int count = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        this.odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        this.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        this.odometry.setOffsets(-6.44, 6.8745);
        this.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.odometry.resetPosAndIMU();
        this.odometry.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        this.robot = new Robot(hardwareMap);

        this.autoDriver = new AutoDriver(robot, odometry);

        waitForStart();

        while (autoDriver.moveTo(new Position(new Distance(24, DistanceUnit.INCH), new Distance(24, DistanceUnit.INCH), new Rotation(180, AngleUnit.DEGREES)), new Distance(0.5, DistanceUnit.INCH), new Rotation(5, AngleUnit.DEGREES)) && opModeIsActive()) {
            codeLoop();
        }
        terminateOpModeNow();
    }

    public void codeLoop(){
        telemetry.addLine("Position: " + odometry.getPosition());
        telemetry.addData("Count: ", count);
        count++;
        telemetry.update();
    }
}
