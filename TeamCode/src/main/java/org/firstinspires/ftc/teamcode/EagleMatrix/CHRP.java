package org.firstinspires.ftc.teamcode.EagleMatrix;

import org.firstinspires.ftc.teamcode.Subsystems.GobuldaPinpointExample;
import org.firstinspires.ftc.teamcode.EagleMatrix.DrivetrainMovements;

// this is where you will put odometry and eaglematrix together yayayayayayayayayayayayaayayayayayayayayay
public class CHRP {
  GobuldaPinpointExample gpe;
  DrivetrainMovements drivetrainMovements;

  public void HeadToY( double distanceY) {
      gpe = gpe;
      drivetrainMovements = drivetrainMovements;

      gpe.getPosition();

      if (gpe.getPosY() > distanceY) ;
      {

          drivetrainMovements.YForward();

      }
      if (gpe.getPosY() < distanceY) ;
      {

          drivetrainMovements.YBackward();
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

              drivetrainMovements.XRight();

          }
          if (gpe.getPosX() < distanceX) ;
          {

              drivetrainMovements.XLeft();
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

                  drivetrainMovements.thetaRight();

              } if (gpe.getHeading() == angle);{
                  drivetrainMovements.stop();
              }

  }

}
