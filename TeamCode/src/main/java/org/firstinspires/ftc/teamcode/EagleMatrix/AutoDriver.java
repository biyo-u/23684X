package org.firstinspires.ftc.teamcode.EagleMatrix;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;

public class AutoDriver {
	Robot robot;
	GoBildaPinpoint pinpoint;
	DriveMovements driveMovements;

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
    public AutoDriver(Robot robot, GoBildaPinpoint pinpoint){
		this.robot = robot;
		this.pinpoint = pinpoint;
		this.driveMovements = new DriveMovements(robot);
	}

	public void moveTo(Vector2D position, double heading){

	}
}
