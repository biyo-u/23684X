package org.firstinspires.ftc.teamcode.EagleMatrix;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Position;

public class AutoDriver {
	Robot robot;
	GoBildaPinpointDriver odometry;
	DriveMovements driveMovements;
	boolean initialized = false;

	// speed of robot = sin(((target location - current location)/target location) * pi) * speed modifier
	// y = sin(xπ) * speed modifier (0.6) (MODIFIER MUST BE LESS OR EQUAL TO 1)
	// repeat for yX, yY, and yHEADING
	public enum MoveOrder {
		HEADING_X_Y,
		HEADING_Y_X,
		X_HEADING_Y,
		Y_HEADING_X,
		X_Y_HEADING,
		Y_X_HEADING,
	}

	public AutoDriver(Robot robot, GoBildaPinpointDriver odometry) {
		this.robot = robot;
		this.odometry = odometry;
		this.driveMovements = new DriveMovements(robot);
	}

	public boolean moveTo(Position position) {
		odometry.update();
		Pose2D currentPosition = odometry.getPosition();

		double x = 0;
		double y = 0;
		double heading = 0;

		if (position.getVector2D().getY() == currentPosition.getY(DistanceUnit.INCH)){
			// No movement needed on y axis
			y = 0;
		} else if (position.getVector2D().getY() > currentPosition.getY(DistanceUnit.INCH)){
			// Move forward
			y = 1;
		} else if (position.getVector2D().getY() < currentPosition.getY(DistanceUnit.INCH)){
			// Move backward
			y = -1;
		}

		if (position.getVector2D().getX() == currentPosition.getX(DistanceUnit.INCH)){
			// No movement on the x axis needed
			x = 0;
		} else if (position.getVector2D().getX() > currentPosition.getX(DistanceUnit.INCH)){
			// Move right
			x = 1;
		} else if (position.getVector2D().getX() < currentPosition.getX(DistanceUnit.INCH)){
			// Move left
			x = -1;
		}

		if (position.getHeading() == currentPosition.getHeading(AngleUnit.DEGREES)){
			// No turning needed
			heading = 0;
		}
		if (position.getHeading() > currentPosition.getHeading(AngleUnit.DEGREES)){
			// No turning needed
			heading = 1;
		}
		if (position.getHeading() < currentPosition.getHeading(AngleUnit.DEGREES)){
			// No turning needed
			heading = -1;
		}

		return !(x > -Constants.doubleErrorThreshold) || !(x < Constants.doubleErrorThreshold) || !(y > -Constants.doubleErrorThreshold) || !(y < Constants.doubleErrorThreshold) || !(heading > -Constants.doubleErrorThreshold) || !(heading < Constants.doubleErrorThreshold);
	}
}
