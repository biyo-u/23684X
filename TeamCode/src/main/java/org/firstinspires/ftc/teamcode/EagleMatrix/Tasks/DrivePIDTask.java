package org.firstinspires.ftc.teamcode.EagleMatrix.Tasks;

import org.firstinspires.ftc.teamcode.EagleMatrix.EaglePID;
import org.firstinspires.ftc.teamcode.Utilities.Task;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.EagleMatrix.DriveMovements.MotorDirection;
import org.firstinspires.ftc.teamcode.EagleMatrix.DriveMovements;


public class DrivePIDTask extends Task {
	EaglePID eaglePIDX;
	EaglePID eaglePIDY;
	EaglePID eaglePIDYaw;
	Robot robot;
	DriveMovements driveMovements;

	@Override
	public void init(Pose2D target) {
		eaglePIDX = new EaglePID(robot.odometry.getPosX());
		eaglePIDY = new EaglePID(robot.odometry.getPosY());
		eaglePIDYaw = new EaglePID(robot.odometry.getHeading());
	}
	@Override
	public boolean run() {
		// loops this until it returns true
		double eaglePIDXValue = eaglePIDX.getPIDOutput(0);
		double eaglePIDYValue = eaglePIDY.getPIDOutput(0);
		double eaglePIDYawValue = eaglePIDYaw.getPIDOutput(0);

		if (eaglePIDXValue == 1234567.89 && eaglePIDYValue == 1234567.89 && eaglePIDYawValue == 1234567.89){
			// stops robot if PID has reached target
			robot.drive.driveMecanumFieldCentric(0,0,0);
			return true;
		} else {
			// move robot until PID has reached target
			robot.drive.driveMecanumFieldCentric(eaglePIDYValue, eaglePIDXValue, eaglePIDYawValue);
			return false;
		}
	}
}
