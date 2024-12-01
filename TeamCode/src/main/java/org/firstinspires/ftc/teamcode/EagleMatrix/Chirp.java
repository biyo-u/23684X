package org.firstinspires.ftc.teamcode.EagleMatrix;

import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.EagleMatrix.DriveMovements;

// this is where you will put odometry and eaglematrix together yayayayayayayayayayayayaayayayayayayayayay
public class Chirp {
	GoBildaPinpointDriver gpe;
	DriveMovements driveMovements;

	public void HeadToY( double distanceY) {
		gpe = gpe;
		driveMovements = driveMovements;

		gpe.getPosition();

		if (gpe.getPosY() > distanceY) ;
		{

			 driveMovements.EagleFlow(0,1,0);

		}
		if (gpe.getPosY() < distanceY) ;
		{

			// driveMovements.move(MotorDirection.BACKWARD);
		}
		if (gpe.getPosY() == distanceY) ;
		{
			driveMovements.EagleStop();
		}
	}

	public void HeadToX(double distanceX) {
		gpe = gpe;
		driveMovements = driveMovements;

		gpe.getPosition();

		if (gpe.getPosX() > distanceX) ;
		{

			// driveMovements.move(MotorDirection.STRAFE_RIGHT);

		}
		if (gpe.getPosX() < distanceX) ;
		{

			// driveMovements.move(MotorDirection.STRAFE_LEFT);
		}
		if (gpe.getPosX() == distanceX) ;
		{
			driveMovements.EagleStop();
		}
	}

	public void TurnClockwise(double angle){
		gpe = gpe;
		driveMovements = driveMovements;

		gpe.getHeading();

		if (gpe.getHeading() > angle);{

			// driveMovements.move(MotorDirection.ROTATE_CLOCKWISE);

		} if (gpe.getHeading() == angle);{
			driveMovements.EagleStop();
		}
	}
}
