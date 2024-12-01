package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

public class Compass {
    private final IMU imu;
    private final GoBildaPinpointDriver odocomp;

    private String model;

    public Compass(IMU imu) {
        this.imu = imu;
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        odocomp = null;
    }

    public double getHeading() {
        if (odocomp != null) {
            model = "Pinpoint IMU";
            odocomp.update();
            return odocomp.getHeading();
        } else {
            model = "REV Hub IMU";
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
    }
    public void resetYaw() {
       if (imu != null) {
           imu.resetYaw();
       } else {
           odocomp.update();
           odocomp.resetPosAndIMU();
       }
    }

    public Compass(GoBildaPinpointDriver goBildaPinpointDriver) {
        this.odocomp = goBildaPinpointDriver;
        imu = null;
    }

    public String getTelemetry() {
        if (imu != null) {
            return String.format(Locale.getDefault(), """
                    Robot Yaw: %f
                    Robot Pitch: %f
                    Robot Roll: %f
                    Model Used: %s""", imu.getRobotYawPitchRollAngles().getYaw(), imu.getRobotYawPitchRollAngles().getPitch(), imu.getRobotYawPitchRollAngles().getRoll(), model);
        } else {
            odocomp.update();
            return String.format(Locale.getDefault(), """
                Robot Yaw: %f
                Model Used: %s""", odocomp.getHeading(), model);
        }
    }
}
