package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Compass;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

public class Robot {
	// Public Subsystems
	public Intake intake;
	public Lift lift;
	public Drive drive;
	public Compass compass;
	public GoBildaPinpointDriver odometry;

	// List todos here
	// TODO: Make auto move diags too
	// TODO: Clean up switch in auto

	public Robot(HardwareMap hardwareMap) {
		// Private Devices
		Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
		Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
		Servo liftServoTiltRight = hardwareMap.get(Servo.class, "liftServoTiltRight");
		Servo liftServoTiltLeft = hardwareMap.get(Servo.class, "liftServoTiltLeft");
		Servo rightHangServo = hardwareMap.get(Servo.class, "rightHangServo");
		Servo leftHangServo = hardwareMap.get(Servo.class, "leftHangServo");
		DcMotor liftMotorLeft = hardwareMap.get(DcMotor.class, "liftMotorLeft");
		DcMotor liftMotorRight = hardwareMap.get(DcMotor.class, "liftMotorRight");
		DcMotor shoulderMotor = hardwareMap.get(DcMotor.class, "shoulderMotor");
		DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
		DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
		DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
		DcMotor rearRight = hardwareMap.get(DcMotor.class, "rearRight");

		IMU imu = hardwareMap.get(IMU.class, "imu");
		GoBildaPinpointDriver odometryComputer = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");

		// Initialize Public Subsystems
//        compass = new Compass(imu);
		compass = new Compass(odometryComputer);
		intake = new Intake(clawServo, wristServo);
		lift = new Lift(liftMotorLeft, liftMotorRight, shoulderMotor, liftServoTiltRight, liftServoTiltLeft, rightHangServo, leftHangServo);
		drive = new Drive(frontLeft, frontRight, rearLeft, rearRight, compass);


		this.odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
		this.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
		this.odometry.setOffsets(-6.44, 6.8745);
		this.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
		this.odometry.resetPosAndIMU();
		this.odometry.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

//        odometry = new Odometry(odometryComputer, compass);
	}
}
