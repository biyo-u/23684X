package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EagleMatrix.DriveMovements;
import org.firstinspires.ftc.teamcode.EagleMatrix.DriveMovements.MotorDirection;
import org.firstinspires.ftc.teamcode.EagleMatrix.LiftMovements;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Distance;
import org.firstinspires.ftc.teamcode.Utilities.Position;
import org.firstinspires.ftc.teamcode.Utilities.Rotation;


@Autonomous(name = "AutoDiagnosticsX", group = Constants.GroupNames.Autonomous, preselectTeleOp = "TeleOp")
public class Autonomosussy extends OpMode {
    private Robot robot; // imports robot hardwareMap class
    private GoBildaPinpointDriver odometry; // imports robot odometry class
    private DriveMovements driveMovements; // imports EagleMatrix movement class for drivetrain
    private LiftMovements liftMovements; // imports EagleMatrix movement class for lift and intakes

    public Position migration = new Position(new Distance(0, DistanceUnit.INCH), new Distance(10, DistanceUnit.INCH), new Rotation(180, AngleUnit.DEGREES)); // target position
    int counter; // counter to ensure AUTO program is active and running loops
    public boolean GoalMet = false; // checks to see if goal (zetaTranslation) has been reached

    // lines 31 to 36 create all the doubles that will be used to compare due to the Double.compare function not working with the direct doubles
    double zetaY = 0; // robot's current X position cast to a double
    double zetaY2 = 0; // robot's target X position cast to a double
    double zetaX = 0; // robot's current Y position cast to a double
    double zetaX2 = 0; // robot's target Y position cast to a double
    double zetaHeading = 0; // robot's current heading cast to a double
    double zetaHeading2 = 0; // robot's target heading cast to a double

    @Override
    public void init() {
        // lines 35-45 initialise all imports so they are not rendered as "null", which when it happens creates the fatal NullPointerException error.
        this.odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        this.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        this.odometry.setOffsets(-6.44, 6.8745);
        this.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.odometry.resetPosAndIMU();
        this.odometry.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        this.robot = new Robot(hardwareMap);
        this.driveMovements = new DriveMovements(robot);
        this.liftMovements = new LiftMovements(robot);
        this.migration = new Position(new Distance(10, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH), new Rotation(0, AngleUnit.DEGREES));

        GoalMet = false; // resets goal to false
        telemetry.addData("Hardware Status", "initialised"); // prints on driver station that all hardware is initialised

        zetaY = odometry.getPosY(); // sets zetaPrime
        zetaY2 = migration.getY();
        zetaX = odometry.getPosX(); // sets zetaPrime
        zetaX2 = migration.getX();
        zetaHeading = odometry.getHeading(); // sets zetaPrime
        zetaHeading2 = migration.getHeading();
    }

    @Override
    public void loop() {

        if (Double.compare(zetaY, zetaY2) > 0.2) {
            // if compare returns a negative value, X1 > X2
            telemetry.addData("AUTO STATUS", "OVERSHOT");
            driveMovements.move(MotorDirection.BACKWARD);

        } else if (Double.compare(zetaY, zetaY2) < -0.2 && !GoalMet) {
            // if compare returns a positive value, X1 < X2
            telemetry.addData("AUTO STATUS", "INCOMPLETE");
            driveMovements.move(MotorDirection.FORWARD);

        } else if (Double.compare(zetaY, zetaY2) < 0.2 && Double.compare(zetaY, zetaY2) > -0.2) {
            // if compare returns a zero value, X1 == X2
            telemetry.addData("AUTO STATUS", "COMPLETE");
            driveMovements.move(MotorDirection.STOP);
            liftMovements.ClawOpen();
            GoalMet = true;

        } else {
            telemetry.addData("AUTO STATUS", "ERROR");
        }

        telemetry.addLine("ZETA PRIME LOCATIONS");
        telemetry.addData("TargetY (Forward, Backward) (INCHES)", migration.getY());
        telemetry.addData("CurrentY (Forward, Backward)", odometry.getPosY());
        telemetry.addData("TargetX (Left, Right) (INCHES)", migration.getX());
        telemetry.addData("CurrentX (Left, Right)", odometry.getPosX());
        telemetry.addData("TargetHeading (Rotation) (DEGREES)", migration.getHeading());
        telemetry.addData("CurrentHeading (Rotation)", odometry.getHeading());

        telemetry.addLine("AUTO STATUS");
        telemetry.addData("COUNT", counter);
        counter++;
        telemetry.update();
    }
}
