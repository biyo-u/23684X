package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

import java.util.Timer;
import java.util.TimerTask;

@Autonomous(name = "AUTO: EagleMatrix 0.2.8", group = Constants.GroupNames.Autonomous, preselectTeleOp = "TeleOp")
public class Auto extends OpMode {
	private Robot robot; // imports robot hardwareMap class
	private GoBildaPinpointDriver odometry; // imports robot odometry class
	private DriveMovements driveMovements; // imports EagleMatrix movement class for drivetrain
	private LiftMovements liftMovements; // imports EagleMatrix movement class for lift and intakes

	// TODO: Ensure these variables will not print out NullPointerExceptions (1)
	public Position migration = new Position(new Distance(0, DistanceUnit.INCH), new Distance(20, DistanceUnit.INCH), new Rotation(180, AngleUnit.DEGREES)); // target position
	int counter; // counter to ensure AUTO program is active and running loops
	public boolean GoalMet = false; // checks to see if goal (zetaTranslation) has been reached

	// lines 31 to 36 create all the numerical variables that will be used to compare due to the Double.compare function not working with the direct doubles
	double zetaY = 0; // robot's current X position cast to a double
	double zetaY2 = 0; // robot's target X position cast to a double
	double zetaX = 0; // robot's current Y position cast to a double
	double zetaX2 = 0; // robot's target Y position cast to a double
	double zetaHeading = 0; // robot's current heading cast to a double
	double zetaHeading2 = 0; // robot's target heading cast to a double

	// lines 45-47 create all the string variables that will be used to a) create the different values for each switch line of code, and keep track of AUTO as it runs
	String actionCounter = "PREP TO SCORE"; // robot's current action, String (text) value for switch block
	String autoStage = null; // robot's current action, String (text) value for telemetry
	String autoStatus = null; // robot's current action status, String (text) value for telemetry

	@Override
	public void init() {
		// lines 48-54 initialise all information related to the odometry pods so when in use will properly function and not print out NullPointerExceptions.
		this.odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
		this.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
		this.odometry.setOffsets(-6.44, 6.8745);
		this.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
		this.odometry.resetPosAndIMU();
		this.odometry.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

		// lines 56-59 initialise all other hardware imports and the movement imports so they are not rendered as "null", which when it happens creates the fatal NullPointerException error.
		this.robot = new Robot(hardwareMap);
		this.driveMovements = new DriveMovements(robot);
		this.liftMovements = new LiftMovements(robot);
		this.migration = new Position(new Distance(0, DistanceUnit.INCH), new Distance(20, DistanceUnit.INCH), new Rotation(0, AngleUnit.DEGREES));

		// stops and rests the lift motor encoders
		this.robot.lift.getLiftMotorRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.robot.lift.getLiftMotorLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		// TODO: Ensure these variables will not print out NullPointerExceptions (2)
		GoalMet = false; // resets goals boolean to false
		telemetry.addData("Hardware Status", "initialised"); // prints on driver station that all hardware is initialised

		zetaY = odometry.getPosY(); // gets zetaPrime's current Y
		zetaY2 = migration.getY(); // sets zetaPrime's target Y
		zetaX = odometry.getPosX(); // get zetaPrime's current X
		zetaX2 = migration.getX(); // sets zetaPrime's target X
		zetaHeading = odometry.getHeading(); // gets zetaPrime's current heading
		zetaHeading2 = migration.getHeading(); // sets zetaPrime's target heading
	}

	@Override
	public void loop() {

		switch (actionCounter) {

			case "PREP TO SCORE": // PREPARES ROBOT LIFT AND INTAKE TO SCORING POSITION
				// TODO: Ensure this code will not print out NullPointerExceptions or issues (3)

				autoStage = "STAGE: PREP TO SCORE";

				Timer timertwo = new Timer("Timer");

				TimerTask preptwo = new TimerTask() {
					@Override
					public void run() {
						liftMovements.ClawClose();
						liftMovements.WristOut();
					}
				};

				TimerTask prepone = new TimerTask() {
					@Override
					public void run() {
						liftMovements.ShoulderUp();
						timertwo.schedule(preptwo, 3000);
					}
				};

				liftMovements.LiftScoreChamber();
				timertwo.schedule(prepone,1000);

				actionCounter = "MOVE TO SUBMERSIBLE";
				break;

			case "MOVE TO SUBMERSIBLE": // ROBOT DRIVES TO SUBMERSIBLE POSITION
				// TODO: Run Autonomosussy to ensure this code will not print out NullPointerExceptions (4)

				autoStage = "STAGE: MOVE TO SUBMERSIBLE";

				if (Double.compare(zetaY, zetaY2) > 0.2) {
					// if compare returns a negative value, value1 > value2
					autoStatus = "OVERSHOT";
					driveMovements.move(MotorDirection.BACKWARD);

				} else if (Double.compare(zetaY, zetaY2) < -0.2 && !GoalMet) {
					// if compare returns a positive value, value1 < value2
					autoStatus = "INCOMPLETE";
					driveMovements.move(MotorDirection.FORWARD);

				} else if (Double.compare(zetaY, zetaY2) < 0.2 && Double.compare(zetaY, zetaY2) > -0.2) {
					// if compare returns a zero value, value1 == value2
					autoStatus = "COMPLETE";
					driveMovements.move(MotorDirection.STOP);
					liftMovements.ClawOpen();
					GoalMet = true;

				} else {
					autoStatus = "ERROR";
				}
				actionCounter = "PLACE PRELOADED SPECIMEN";
				break;

			case "PLACE PRELOADED SPECIMEN": // ROBOT PLACES PRELOADED SPECIMEN ON HIGH CHAMBER
				// TODO: Ensure this code will not print out NullPointerExceptions or issues (5)

				autoStage = "STAGE: PLACE PRELOADED SPECIMEN";

				Timer timerone = new Timer("Timer");

				TimerTask waittwo = new TimerTask() {
					@Override
					public void run() {
						liftMovements.LiftScoreChamber();
					}
				};

				TimerTask waitone = new TimerTask() {
					@Override
					public void run() {
						liftMovements.ClawOpen();
						timerone.schedule(waittwo, 2000);
					}
				};

				liftMovements.LiftPullDown();
				timerone.schedule(waitone,3000);

				actionCounter = "LOWER LIFT";
				break;


			case "LOWER LIFT": // MOVES ROBOT BACK AND LOWERS DOWN THE LIFT
				// TODO: Ensure this code will not print out NullPointerExceptions or issues (6)

				autoStage = "STAGE: LOWER LIFT";

				boolean NextGoalMet = false;
				Position nextGoal = new Position(new Distance(0, DistanceUnit.INCH), new Distance(-10, DistanceUnit.INCH), new Rotation(0, AngleUnit.DEGREES));

				double zetaY2_2 = odometry.getPosY();
				double zetaY2_GOAL = nextGoal.getY();

				// TODO: Run Autonomosussy to ensure this code will not print out NullPointerExceptions (8)
				if (Double.compare(zetaY2_2, zetaY2_GOAL) > 0.2) {
					// if compare returns a negative value, value1 > value2
					autoStatus = "OVERSHOT, BUT IT'S ALRIGHT";
					driveMovements.move(MotorDirection.STOP);
					liftMovements.LiftLower();
					NextGoalMet = true;
				} else if (Double.compare(zetaY2_2, zetaY2_GOAL) < -0.2 && NextGoalMet == false) {
					// if compare returns a positive value, value1 < value2
					autoStatus = "INCOMPLETE";
					driveMovements.move(MotorDirection.BACKWARD);
				} else if (Double.compare(zetaY2_2, zetaY2_GOAL) < 0.2 && Double.compare(zetaY2_2, zetaY2_GOAL) > -0.2) {
					// if compare returns a zero value, value1 == value2
					autoStatus = "COMPLETE";
					driveMovements.move(MotorDirection.STOP);
					liftMovements.LiftLower();
					NextGoalMet = true;
				}

				if (NextGoalMet == true) {
					actionCounter = "MOVE TO OBSERVATION ZONE";
				}
				break;

			case "MOVE TO OBSERVATION ZONE": // MOVES ROBOT TO OBSERVATION ZONE FROM SUBMERSIBLE POSITION
				// TODO: Ensure these variables will not print out NullPointerExceptions (7)

				autoStage = "STAGE: MOVE TO OBSERVATION ZONE";

				boolean FinalGoalMet = false;
				Position observationZone = new Position(new Distance(30, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH), new Rotation(0, AngleUnit.DEGREES));

				double zetaX2_3 = observationZone.getX();
				zetaX2 = observationZone.getX();

				// TODO: Run Autonomosussy to ensure this code will not print out NullPointerExceptions (8)
				if (Double.compare(zetaY, zetaX2_3) > 0.2) {
					// if compare returns a negative value, value1 > value2
					autoStatus = "OVERSHOT, BUT IT'S ALRIGHT";
					driveMovements.move(MotorDirection.STOP);
					FinalGoalMet = true;
				} else if (Double.compare(zetaY, zetaX2_3) < -0.2 && FinalGoalMet == false) {
					// if compare returns a positive value, value1 < value2
					autoStatus = "INCOMPLETE";
					driveMovements.move(MotorDirection.STRAFE_RIGHT);
				} else if (Double.compare(zetaY, zetaX2_3) < 0.2 && Double.compare(zetaY, zetaX2_3) > -0.2) {
					// if compare returns a zero value, value1 == value2
					autoStatus = "COMPLETE";
					driveMovements.move(MotorDirection.STOP);
					FinalGoalMet = true;
				}

				if (FinalGoalMet == true) {
					actionCounter = "EXEUNT";
				}
				break;

			case "EXEUNT": // ENDS THE SWITCH STATEMENT ONCE ALL CASES ARE COMPLETED
				// TODO: Ensure a 'default' state is not required for this switch statement (9)

				autoStage = "STAGE: EXEUNT";
				autoStatus = "AUTONOMOUS PROCESS COMPLETE, PLEASE PREPARE FOR TELEOP.";
				break;

			default: // IF actionCounter IS NOT DEFINED OR DEFINED TO A UNDEFINED STATE, PRINT ERROR
				autoStage = "SWITCH ERROR";
		}

		// TODO: Ensure these telemetry lines will not print out NullPointerExceptions (10)
		telemetry.addLine("ZETA PRIME LOCATIONS"); // heading title
		telemetry.addData("TargetY (Forward, Backward) (INCHES)", migration.getY()); // robot's target Y position
		telemetry.addData("CurrentY (Forward, Backward)", odometry.getPosY()); // robot's current Y position
		telemetry.addData("TargetX (Left, Right) (INCHES)", migration.getX()); // robot's target X position
		telemetry.addData("CurrentX (Left, Right)", odometry.getPosX()); // robot's current X position
		telemetry.addData("TargetHeading (Rotation) (DEGREES)", migration.getHeading()); // robot's target heading
		telemetry.addData("CurrentHeading (Rotation)", odometry.getHeading()); // robot's current heading

		telemetry.addLine("AUTO DIAGNOSTICS"); // heading title
		telemetry.addData("COUNT", counter); // counter
		telemetry.addData("AUTO STAGE", autoStage); // stage auto is in
		telemetry.addData("AUTO STATUS", autoStatus); // status action is in
		telemetry.addLine("EagleMatrix 0.2.8."); // library version title
		counter++; // update counter for ever loop
		telemetry.update(); // updates odometry every loop
	}
}
// har har line 269... bro i spent 3 whole days writing this, i will actually cry so hard if it doesn't work. i'd like to thank beyonce, pomodoro timers, and w3schools for making EagleMatrix 0.2.0 possible