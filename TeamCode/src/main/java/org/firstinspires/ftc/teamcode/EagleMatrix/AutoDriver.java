package org.firstinspires.ftc.teamcode.EagleMatrix;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utilities.Position;

public class AutoDriver {
	Robot robot;
	GoBildaPinpointDriver odometry;
	DriveMovements driveMovements;
	boolean initialized = false;

	public enum MoveOrder {
		HEADING_X_Y,
		HEADING_Y_X,
		X_HEADING_Y,
		Y_HEADING_X,
		X_Y_HEADING,
		Y_X_HEADING,
	}

    public AutoDriver(Robot robot, GoBildaPinpointDriver odometry){
		this.robot = robot;
		this.odometry = odometry;
		this.driveMovements = new DriveMovements(robot);
	}

	public boolean moveTo(Position position){
		odometry.update();
		Pose2D currentPosition = odometry.getPosition();

		if (!initialized){

			if (position.getVector2D().getY() == currentPosition.getY(DistanceUnit.INCH)){
				// No movement needed, may need to rotate later
			} else if (position.getVector2D().getY() > currentPosition.getY(DistanceUnit.INCH)){
				// Move forward
			} else if (position.getVector2D().getY() < currentPosition.getY(DistanceUnit.INCH)){
				// Move backward
			}

			if (position.getVector2D().getX() == currentPosition.getX(DistanceUnit.INCH)){
				// No movement needed, may need to rotate later
			} else if (position.getVector2D().getX() > currentPosition.getX(DistanceUnit.INCH)){
				// Move right
			} else if (position.getVector2D().getX() < currentPosition.getX(DistanceUnit.INCH)){
				// Move left
			}
		}

		return true;
	}
}
