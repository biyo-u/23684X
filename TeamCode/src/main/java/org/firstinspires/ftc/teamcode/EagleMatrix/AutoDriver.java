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
	Pose2D startPosition;
    boolean waypoint = false;

	// speed of robot = sin(((target location - current location)/target location) * pi) * speed modifier
	// y = sin(xπ) * speed modifier (0.6) (MODIFIER MUST BE LESS OR EQUAL TO 1)
	// repeat for yX, yY, and yHEADING


	// NEW SPEED (in LaTeX):

	//y\ =\frac{\left(\ \frac{1}{\sqrt{2\pi}}e^{-\frac{1}{2}\left(\frac{x-0.5}{0.15}\right)^{2}}\right)}{1}
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

	public boolean moveTo(Position position, double sensitivity) {
		odometry.update();
		Pose2D currentPosition = odometry.getPosition();
        if (!waypoint){
            waypoint = true;
            startPosition = currentPosition;
        }

		if (!initialized) {

		}

		double x = hummingbird(startPosition.getX(DistanceUnit.INCH), currentPosition.getX(DistanceUnit.INCH), position.getVector2D().getX());
		double y = hummingbird(startPosition.getY(DistanceUnit.INCH), currentPosition.getY(DistanceUnit.INCH), position.getVector2D().getY());
		double heading = hummingbird(startPosition.getHeading(AngleUnit.DEGREES), currentPosition.getHeading(AngleUnit.DEGREES), position.getHeading());

		double yDifferenceFromTarget = Math.abs(currentPosition.getY(DistanceUnit.INCH)) - Math.abs(position.getVector2D().getY());
		if (yDifferenceFromTarget > -sensitivity && yDifferenceFromTarget < sensitivity){
			// No movement needed on y axis
			y = 0;
		} else if (position.getVector2D().getY() > currentPosition.getY(DistanceUnit.INCH)){
			// Move forward
			y = 1;
		} else if (position.getVector2D().getY() < currentPosition.getY(DistanceUnit.INCH)){
			// Move backward
			y = -1;
		}

		double xDifferenceFromTarget = Math.abs(currentPosition.getX(DistanceUnit.INCH)) - Math.abs(position.getVector2D().getX());
		if (xDifferenceFromTarget > -sensitivity && xDifferenceFromTarget < sensitivity){
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
		} else if (position.getHeading() > currentPosition.getHeading(AngleUnit.DEGREES)){
			// No turning needed
			heading = 1;
		} else if (position.getHeading() < currentPosition.getHeading(AngleUnit.DEGREES)){
			// No turning needed
			heading = -1;
		}

        driveMovements.EagleFlow(x, y, heading);

		return !(x > -Constants.doubleErrorThreshold) || !(x < Constants.doubleErrorThreshold) || !(y > -Constants.doubleErrorThreshold) || !(y < Constants.doubleErrorThreshold) || !(heading > -Constants.doubleErrorThreshold) || !(heading < Constants.doubleErrorThreshold);
	}

    public double hummingbird(double startingPosition, double currentPosition, double endPosition){
        double robotspeed = 0.0;
        double percentPathComplete = currentPosition/(endPosition-startingPosition);
        //y\ =\frac{\left(\ \frac{1}{\sqrt{2\pi}}e^{-\frac{1}{2}\left(\frac{x-0.5}{0.15}\right)^{2}}\right)}{1}
        robotspeed = (1 / Math.sqrt(2 * Math.PI)) * Math.exp(-Math.pow(percentPathComplete-0.5 / 0.15, 2)); // calculate speed on a bell curve
        return robotspeed;
    }
}
