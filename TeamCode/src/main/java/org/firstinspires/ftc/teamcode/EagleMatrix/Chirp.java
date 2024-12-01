package org.firstinspires.ftc.teamcode.EagleMatrix;

import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.EagleMatrix.DriveMovements.MotorDirection;

// this is where you will put odometry and eaglematrix together yayayayayayayayayayayayaayayayayayayayayay
public class Chirp {
	GoBildaPinpoint gpe;
	DriveMovements driveMovements;

	public void HeadToY( double distanceY) {
		gpe = gpe;
		driveMovements = driveMovements;

		gpe.getPosition();

		if (gpe.getPosY() > distanceY) ;
		{

			driveMovements.move(MotorDirection.FORWARD);

		}
		if (gpe.getPosY() < distanceY) ;
		{

			driveMovements.move(MotorDirection.BACKWARD);
		}
		if (gpe.getPosY() == distanceY) ;
		{
			driveMovements.stop();
		}
	}

	public void HeadToX(double distanceX) {
		gpe = gpe;
		driveMovements = driveMovements;

		gpe.getPosition();

		if (gpe.getPosX() > distanceX) ;
		{

			driveMovements.move(MotorDirection.STRAFE_RIGHT);

		}
		if (gpe.getPosX() < distanceX) ;
		{

			driveMovements.move(MotorDirection.STRAFE_LEFT);
		}
		if (gpe.getPosX() == distanceX) ;
		{
			driveMovements.stop();
		}
	}

	public void TurnClockwise(double angle){
		gpe = gpe;
		driveMovements = driveMovements;

		gpe.getHeading();

		if (gpe.getHeading() > angle);{

			driveMovements.move(MotorDirection.ROTATE_CLOCKWISE);

		} if (gpe.getHeading() == angle);{
			driveMovements.stop();
		}
	}
}
