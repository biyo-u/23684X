package org.firstinspires.ftc.teamcode.EagleMatrix;

import org.firstinspires.ftc.teamcode.EagleMatrix.DrivetrainMovements.MotorDirection;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;

// this is where you will put odometry and eaglematrix together yayayayayayayayayayayayaayayayayayayayayay
public class AutoDriver {
	GoBildaPinpoint pinpoint;
	DrivetrainMovements drivetrainMovements;
	public enum MoveOrder {
		HEADING_X_Y,
		HEADING_Y_X,
		X_HEADING_Y,
		Y_HEADING_X,
		X_Y_HEADING,
		Y_X_HEADING,
	}

	public void moveTo(Vector2D position, double heading){

	}
}
