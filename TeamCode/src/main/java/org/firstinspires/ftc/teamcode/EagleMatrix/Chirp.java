package org.firstinspires.ftc.teamcode.EagleMatrix;

import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.EagleMatrix.DrivetrainMovements.MotorDirection;

// this is where you will put odometry and eaglematrix together yayayayayayayayayayayayaayayayayayayayayay
public class Chirp {
	GoBildaPinpoint gpe;
	DrivetrainMovements drivetrainMovements;

	public void HeadToY( double distanceY) {
		gpe = gpe;
		drivetrainMovements = drivetrainMovements;

		gpe.getPosition();

		if (gpe.getPosY() > distanceY) ;
		{

			drivetrainMovements.move(MotorDirection.FORWARD);

		}
		if (gpe.getPosY() < distanceY) ;
		{

			drivetrainMovements.move(MotorDirection.BACKWARD);
		}
		if (gpe.getPosY() == distanceY) ;
		{
			drivetrainMovements.stop();
		}
	}

	public void HeadToX(double distanceX) {
		gpe = gpe;
		drivetrainMovements = drivetrainMovements;

		gpe.getPosition();

		if (gpe.getPosX() > distanceX) ;
		{

			drivetrainMovements.move(MotorDirection.STRAFE_RIGHT);

		}
		if (gpe.getPosX() < distanceX) ;
		{

			drivetrainMovements.move(MotorDirection.STRAFE_LEFT);
		}
		if (gpe.getPosX() == distanceX) ;
		{
			drivetrainMovements.stop();
		}
	}

	public void TurnClockwise(double angle){
		gpe = gpe;
		drivetrainMovements = drivetrainMovements;

		gpe.getHeading();

		if (gpe.getHeading() > angle);{

			drivetrainMovements.move(MotorDirection.ROTATE_CLOCKWISE);

		} if (gpe.getHeading() == angle);{
			drivetrainMovements.stop();
		}
	}
}
