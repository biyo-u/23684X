package org.firstinspires.ftc.teamcode.EagleMatrix;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Distance;
import org.firstinspires.ftc.teamcode.Utilities.Position;
import org.firstinspires.ftc.teamcode.Utilities.Rotation;

public class AutoDriver {
	Robot robot;
	GoBildaPinpointDriver odometry;
	DriveMovements driveMovements;
	boolean initialized = false;
	Position startPosition;
	Position originalMovementDirections;

	public AutoDriver(Robot robot, GoBildaPinpointDriver odometry) {
		this.robot = robot;
		this.odometry = odometry;
		this.driveMovements = new DriveMovements(robot);
	}

	public boolean moveTo(Position position, Distance distanceSensitivity, Rotation headingSensitivity) {
		odometry.update();
		Pose2D currentPositionPose = odometry.getPosition();
		Position currentPosition = new Position(new Distance(currentPositionPose.getX(DistanceUnit.INCH), DistanceUnit.INCH), new Distance(currentPositionPose.getY(DistanceUnit.INCH), DistanceUnit.INCH), new Rotation(currentPositionPose.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES));
		originalMovementDirections = new Position(new Distance(0, DistanceUnit.INCH), new Distance(0, DistanceUnit.INCH), new Rotation(0, AngleUnit.DEGREES));

		if (!initialized) {
			startPosition = currentPosition;
			if (startPosition.getX() == position.getX()){
				originalMovementDirections.setX(new Distance(0, DistanceUnit.INCH));
			} else if (startPosition.getX() > position.getX()){
				originalMovementDirections.setX(new Distance(-1, DistanceUnit.INCH));
			} else if (startPosition.getX() < position.getX()){
				originalMovementDirections.setX(new Distance(1, DistanceUnit.INCH));
			}

			if (startPosition.getY() == position.getY()){
				originalMovementDirections.setY(new Distance(0, DistanceUnit.INCH));
			} else if (startPosition.getY() > position.getY()){
				originalMovementDirections.setY(new Distance(-1, DistanceUnit.INCH));
			} else if (startPosition.getY() < position.getY()){
				originalMovementDirections.setY(new Distance(1, DistanceUnit.INCH));
			}

			if (startPosition.getHeading() == position.getHeading()){
				originalMovementDirections.setHeading(0);
			} else if (startPosition.getY() > position.getY()){
				originalMovementDirections.setHeading(-1);
			} else if (startPosition.getY() < position.getY()){
				originalMovementDirections.setHeading(1);
			}
		}

		double x = 0;
		double y = 0;
		double heading = 0;

		double yDifferenceFromTarget = Math.abs(currentPosition.getY()) - Math.abs(position.getVector2D().getY());
		if (yDifferenceFromTarget > -distanceSensitivity.getDistance() && yDifferenceFromTarget < distanceSensitivity.getDistance()){
			// No movement needed on y axis
			y = 0;
		} else if (position.getY() > currentPosition.getY()){
			// Move forward
			y = 1;
		} else if (position.getY() < currentPosition.getY()){
			// Move backward
			y = -1;
		}

		double xDifferenceFromTarget = Math.abs(currentPosition.getX()) - Math.abs(position.getVector2D().getX());
		if (xDifferenceFromTarget > -distanceSensitivity.getDistance() && xDifferenceFromTarget < distanceSensitivity.getDistance()){
			// No movement on the x axis needed
			x = 0;
		} else if (position.getX() > currentPosition.getX()){
			// Move right
			x = 1;
		} else if (position.getX() < currentPosition.getX()){
			// Move left
			x = -1;
		}

		double headingDifferenceFromTarget = Math.abs(currentPosition.getHeading()) - Math.abs(position.getHeading());
		if (headingDifferenceFromTarget > -headingSensitivity.getAngle() && headingDifferenceFromTarget < headingSensitivity.getAngle()){
			// No turning needed
			heading = 0;
		} else if (position.getHeading() > currentPosition.getHeading()){
			// Turn more
			heading = 1;
		} else if (position.getHeading() < currentPosition.getHeading()){
			// Turn less
			heading = -1;
		}

		/* Speed Formulas (in LaTeX):
		Normal: y=\frac{\left(\ \frac{1}{\sqrt{2\pi}}e^{-\frac{1}{2}\left(\frac{x-0.5}{0.3}\right)^{2}}\right)}{1}
		Overshot: y=-\frac{\left(\ \frac{1}{\sqrt{2\pi}}e^{-\frac{1}{2}\left(\frac{x-1.5}{0.3}\right)^{2}}\right)}{1}
		 */

		double xProgress = (position.getX() - startPosition.getX())/startPosition.getX();
		double xSpeed = 0;
		if (xProgress < 1){
			xSpeed = ((1/(Math.sqrt(2*Math.PI)))*Math.pow(Math.E,(-1.0/2.0*(Math.pow(((xProgress - 0.5) / 0.2), 2.0))))*2.506628);
		} else {
			xSpeed = -((1/(Math.sqrt(2*Math.PI)))*Math.pow(Math.E,(-1.0/2.0*(Math.pow(((xProgress - 1.5) / 0.2), 2.0))))*2.506628);
		}

		double yProgress = (position.getY() - startPosition.getY())/startPosition.getY();
		double ySpeed = 0;
		if (xProgress < 1){
			ySpeed = ((1/(Math.sqrt(2*Math.PI)))*Math.pow(Math.E,(-1.0/2.0*(Math.pow(((yProgress - 0.5) / 0.2), 2.0))))*2.506628);
		} else {
			ySpeed = -((1/(Math.sqrt(2*Math.PI)))*Math.pow(Math.E,(-1.0/2.0*(Math.pow(((yProgress - 1.5) / 0.2), 2.0))))*2.506628);
		}

		double headingProgress = (position.getHeading() - startPosition.getHeading())/startPosition.getHeading();
		double headingSpeed = 0;
		if (xProgress < 1){
			headingSpeed = ((1/(Math.sqrt(2*Math.PI)))*Math.pow(Math.E,(-1.0/2.0*(Math.pow(((headingProgress - 0.5) / 0.2), 2.0))))*2.506628);
		} else {
			headingSpeed = -((1/(Math.sqrt(2*Math.PI)))*Math.pow(Math.E,(-1.0/2.0*(Math.pow(((headingProgress - 1.5) / 0.2), 2.0))))*2.506628);
		}

		double xPower = x * xSpeed;
		double yPower = y * ySpeed;
		double headingPower = heading * headingSpeed;

		driveMovements.EagleFlow(xPower, yPower, headingPower);

		// Return False to end the while when all conditions are met
		return !(x > -Constants.doubleErrorThreshold) || !(x < Constants.doubleErrorThreshold) || !(y > -Constants.doubleErrorThreshold) || !(y < Constants.doubleErrorThreshold) || !(heading > -Constants.doubleErrorThreshold) || !(heading < Constants.doubleErrorThreshold);
	}
}
