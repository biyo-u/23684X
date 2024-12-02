package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
public class Auto extends OpMode {
    private Robot robot;
    private GoBildaPinpointDriver odometry;
    private AutoDriver autoDriver;

    @Override
    public void init() {
        this.odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        this.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        this.odometry.setOffsets(-6.44, 6.8745);
        this.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.odometry.resetPosAndIMU();
        this.odometry.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        this.autoDriver = new AutoDriver(robot, odometry);
    }

    @Override
    public void loop() {
        while (autoDriver.moveTo(new Position(10, 0, 0, AngleUnit.DEGREES), 0.5)) {
            codeLoop();
        }
        terminateOpModeNow();
    }

    public void codeLoop(){
        telemetry.addLine("Position: " + odometry.getPosition());
    }
}
