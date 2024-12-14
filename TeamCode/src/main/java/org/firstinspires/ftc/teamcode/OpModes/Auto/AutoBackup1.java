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
import org.firstinspires.ftc.teamcode.Utilities.Old.Distance;
import org.firstinspires.ftc.teamcode.Utilities.Old.Position;
import org.firstinspires.ftc.teamcode.Utilities.Old.Rotation;


@Autonomous(name = "AutoBackup", group = Constants.GroupNames.Autonomous, preselectTeleOp = "TeleOp")
public class AutoBackup1 extends OpMode {
	private Robot robot; // imports robot hardwareMap class
	private GoBildaPinpointDriver odometry; // imports robot odometry class
	private DriveMovements driveMovements; // imports EagleMatrix movement class for drivetrain
	private LiftMovements liftMovements; // imports EagleMatrix movement class for lift and intakes

	public Position migration = new Position(new Distance(0, DistanceUnit.INCH), new Distance(24, DistanceUnit.INCH), new Rotation(180, AngleUnit.DEGREES)); // target position
	int counter; // counter to ensure AUTO program is active and running loops
	public boolean GoalMet = false; // checks to see if goal (zetaTranslation) has been reached

	// lines 31 to 36 create all the doubles that will be used to compare due to the Double.compare function not working with the direct doubles
	double zetaY = 0; // robot's current X position cast to a double
	double zetaY2 = 0; // robot's target X position cast to a double
	double zetaX = 0; // robot's current Y position cast to a double
	double zetaX2 = 0; // robot's target Y position cast to a double
	double zetaHeading = 0; // robot's current heading cast to a double
	double zetaHeading2 = 0; // robot's target heading cast to a double

	double correctionValue = 5.0;

	String autoStatus = null;

	@Override
	public void init() {
		// lines 35-45 initialise all imports so they are not rendered as "null", which when it happens creates the fatal NullPointerException error.
		this.odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
		this.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
		this.odometry.setOffsets(-6.44, 6.8745);
		this.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
		this.odometry.resetPosAndIMU();
		this.odometry.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

		this.robot = new Robot(hardwareMap);
		this.driveMovements = new DriveMovements(robot);
		this.liftMovements = new LiftMovements(robot);
		this.migration = new Position(new Distance(0, DistanceUnit.INCH), new Distance(36, DistanceUnit.INCH), new Rotation(0, AngleUnit.DEGREES));

		GoalMet = false; // resets goal to false
		telemetry.addData("Hardware Status", "initialised"); // prints on driver station that all hardware is initialised

		zetaY = -odometry.getPosY(); // sets zetaPrime
		zetaY2 = migration.getY();
		zetaX = odometry.getPosX(); // sets zetaPrime
		zetaX2 = migration.getX();
		zetaHeading = odometry.getHeading(); // sets zetaPrime
		zetaHeading2 = migration.getHeading();
	}

	@Override
	public void loop() {
		odometry.update();

		Pose2D zetaPosition = odometry.getPosition();
		zetaY = zetaPosition.getY(DistanceUnit.INCH); // sets zetaPrime
		zetaX = zetaPosition.getX(DistanceUnit.INCH); // sets zetaPrime
		zetaHeading = zetaPosition.getHeading(AngleUnit.DEGREES); // sets zetaPrime

		if (zetaY - migration.getY() > correctionValue) {
			// if compare returns a negative value, X1 > X2
			autoStatus = "OVERSHOT";
			driveMovements.move(MotorDirection.BACKWARD);

		} else if (zetaY - migration.getY() < -correctionValue) {
			// if compare returns a positive value, X1 < X2
			autoStatus = "INCOMPLETE";
			driveMovements.move(MotorDirection.FORWARD);

//		} else if (Double.compare(odometry.getPosY(), migration.getY()) < correctionValue && Double.compare(odometry.getPosY(), migration.getY()) > -correctionValue) {
		} else { //if (odometry.getPosY() - migration.getY() < correctionValue && (Math.abs(odometry.getPosY()) - Math.abs(migration.getY())) > -correctionValue){
			// if compare returns a zero value, X1 == X2
			autoStatus = "COMPLETE";
			driveMovements.move(MotorDirection.STOP);
			liftMovements.ClawOpen();
			GoalMet = true;
		}

		telemetry.addLine("ZETA PRIME LOCATIONS");
		telemetry.addData("TargetY (Forward, Backward) (INCHES)", migration.getY());
		telemetry.addData("CurrentY (Forward, Backward)", zetaY);
		telemetry.addData("TargetX (Left, Right) (INCHES)", migration.getX());
		telemetry.addData("CurrentX (Left, Right)", zetaX);
		telemetry.addData("TargetHeading (Rotation) (DEGREES)", migration.getHeading());
		telemetry.addData("CurrentHeading (Rotation)", zetaHeading);

		telemetry.addLine("AUTO RUNNING INFO");
		telemetry.addData("COUNT", counter);
		telemetry.addData("AUTO STATUS", autoStatus);
		counter++;
		odometry.update();
		telemetry.update();
	}
}